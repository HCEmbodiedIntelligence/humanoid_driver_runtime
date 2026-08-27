// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "humanoid_driver_runtime/ros_topic_robot_driver.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include "pluginlib/class_list_macros.hpp"

namespace hdi = humanoid_driver_interface;

namespace
{

double parseFiniteSeconds(const std::string & key, const std::string & value)
{
  std::size_t consumed = 0;
  const double result = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(result) || result <= 0.0) {
    throw std::invalid_argument(key + " must be a positive finite number of seconds");
  }
  return result;
}

bool allFinite(const std::vector<double> & values)
{
  return std::all_of(
    values.begin(), values.end(), [](const double value) {return std::isfinite(value);});
}

}  // namespace

namespace humanoid_driver_runtime
{

RosTopicRobotDriver::Result RosTopicRobotDriver::attachRosNode(rclcpp::Node & node)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (configured_) {
    return Result::failure(hdi::DriverError::kInvalidState, "attach ROS node before configure");
  }
  if (node_ != nullptr && node_ != &node) {
    return Result::failure(hdi::DriverError::kInvalidState, "ROS node is already attached");
  }
  node_ = &node;
  return Result::success("ROS node attached");
}

RosTopicRobotDriver::Result RosTopicRobotDriver::configure(
  const hdi::DriverConfiguration & configuration)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (node_ == nullptr) {
    return Result::failure(
      hdi::DriverError::kInvalidState, "ROS topic driver requires attachRosNode before configure");
  }
  if (connected_ || active_) {
    return Result::failure(
      hdi::DriverError::kInvalidState, "cannot configure a connected ROS topic driver");
  }
  if (configuration.joints.empty()) {
    return Result::failure(
      hdi::DriverError::kInvalidConfiguration, "at least one joint mapping is required");
  }

  std::string state_topic;
  std::string command_topic;
  double state_timeout_seconds = 0.25;
  double startup_grace_seconds = 3.0;
  try {
    for (const auto & [key, value] : configuration.parameters) {
      if (key == "state_topic") {
        state_topic = value;
      } else if (key == "command_topic") {
        command_topic = value;
      } else if (key == "state_timeout_s") {
        state_timeout_seconds = parseFiniteSeconds(key, value);
      } else if (key == "startup_grace_s") {
        startup_grace_seconds = parseFiniteSeconds(key, value);
      } else {
        throw std::invalid_argument("unknown ROS topic driver parameter '" + key + "'");
      }
    }
  } catch (const std::exception & error) {
    return Result::failure(hdi::DriverError::kInvalidConfiguration, error.what());
  }
  if (state_topic.empty() || command_topic.empty()) {
    return Result::failure(
      hdi::DriverError::kInvalidConfiguration,
      "state_topic and command_topic are required ROS topic driver parameters");
  }

  std::unordered_set<std::string> logical_names;
  std::unordered_set<std::string> vendor_names;
  for (const auto & mapping : configuration.joints) {
    if (mapping.logical_name.empty() || mapping.vendor_name.empty() || mapping.vendor_group.empty() ||
      !std::isfinite(mapping.vendor_to_logical_scale) ||
      std::abs(mapping.vendor_to_logical_scale) < std::numeric_limits<double>::epsilon() ||
      !std::isfinite(mapping.vendor_to_logical_offset_rad))
    {
      return Result::failure(
        hdi::DriverError::kInvalidConfiguration,
        "ROS topic driver mappings require names, a non-zero finite scale, and a finite offset");
    }
    if (!logical_names.insert(mapping.logical_name).second ||
      !vendor_names.insert(mapping.vendor_name).second)
    {
      return Result::failure(
        hdi::DriverError::kInvalidConfiguration,
        "ROS topic driver logical and vendor joint names must be unique");
    }
  }

  try {
    command_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      command_topic, rclcpp::QoS(10).reliable());
    state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      state_topic, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {stateCallback(message);});
  } catch (const std::exception & error) {
    command_publisher_.reset();
    state_subscription_.reset();
    return Result::failure(
      hdi::DriverError::kInternal, "failed to create ROS topic endpoints: " + std::string(error.what()));
  }

  mappings_ = configuration.joints;
  mapping_by_logical_name_.clear();
  mapping_by_vendor_name_.clear();
  for (std::size_t index = 0; index < mappings_.size(); ++index) {
    mapping_by_logical_name_.emplace(mappings_[index].logical_name, index);
    mapping_by_vendor_name_.emplace(mappings_[index].vendor_name, index);
  }
  state_topic_ = std::move(state_topic);
  command_topic_ = std::move(command_topic);
  state_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(state_timeout_seconds));
  startup_grace_ = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::duration<double>(startup_grace_seconds));
  configured_at_ = Clock::now();
  latest_state_ = {};
  last_state_received_ = configured_at_;
  last_feedback_error_.clear();
  configured_ = true;
  connected_ = false;
  active_ = false;
  streaming_ = false;
  stopped_ = true;
  have_state_ = false;
  return Result::success("ROS topic driver configured");
}

RosTopicRobotDriver::Result RosTopicRobotDriver::connect()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!configured_ || active_) {
    return Result::failure(
      hdi::DriverError::kInvalidState, "configure an inactive ROS topic driver before connect");
  }
  if (!state_subscription_ || !command_publisher_) {
    return Result::failure(hdi::DriverError::kInternal, "ROS topic endpoints are unavailable");
  }
  connected_ = true;
  return Result::success("ROS topic endpoints connected");
}

RosTopicRobotDriver::Result RosTopicRobotDriver::disconnect()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_) {
    return Result::failure(
      hdi::DriverError::kInvalidState, "deactivate ROS topic driver before disconnect");
  }
  connected_ = false;
  streaming_ = false;
  stopped_ = true;
  return Result::success("ROS topic endpoints disconnected");
}

RosTopicRobotDriver::Result RosTopicRobotDriver::activate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!connected_ || active_) {
    return Result::failure(
      hdi::DriverError::kInvalidState, "connect an inactive ROS topic driver before activate");
  }
  active_ = true;
  stopped_ = true;
  return Result::success("ROS topic driver active");
}

RosTopicRobotDriver::Result RosTopicRobotDriver::deactivate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_) {
    return Result::success("ROS topic driver already inactive");
  }
  active_ = false;
  streaming_ = false;
  stopped_ = true;
  return Result::success("ROS topic driver inactive");
}

RosTopicRobotDriver::Result RosTopicRobotDriver::readJointState(hdi::JointState & state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || !streaming_) {
    return Result::failure(
      hdi::DriverError::kNotActive, "ROS topic joint stream is not active");
  }
  const auto now = Clock::now();
  if (!have_state_) {
    if (now - configured_at_ < startup_grace_) {
      return Result::failure(
        hdi::DriverError::kNoFeedback, "awaiting first valid ROS joint-state sample on " + state_topic_);
    }
    return Result::failure(
      hdi::DriverError::kCommunication, "no valid ROS joint-state sample received on " + state_topic_);
  }
  if (!stateFreshLocked(now)) {
    return Result::failure(
      hdi::DriverError::kCommunication, "ROS joint-state sample is stale on " + state_topic_);
  }
  state = latest_state_;
  return Result::success();
}

RosTopicRobotDriver::Result RosTopicRobotDriver::startJointStream()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_) {
    return Result::failure(hdi::DriverError::kNotActive, "ROS topic driver is not active");
  }
  streaming_ = true;
  return Result::success("ROS topic joint stream started");
}

RosTopicRobotDriver::Result RosTopicRobotDriver::writeJointCommand(const hdi::JointCommand & command)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || !streaming_) {
    return Result::failure(
      hdi::DriverError::kNotActive, "ROS topic joint stream is not active");
  }
  if (!have_state_) {
    return Result::failure(hdi::DriverError::kNoFeedback, "cannot command before first valid state");
  }
  if (!stateFreshLocked(Clock::now())) {
    return Result::failure(hdi::DriverError::kCommunication, "cannot command with stale joint state");
  }
  const auto result = publishCommandLocked(command);
  if (result) {
    stopped_ = false;
  }
  return result;
}

RosTopicRobotDriver::Result RosTopicRobotDriver::stopJointStream()
{
  std::lock_guard<std::mutex> lock(mutex_);
  streaming_ = false;
  stopped_ = true;
  return Result::success("ROS topic joint stream stopped");
}

RosTopicRobotDriver::Result RosTopicRobotDriver::stopAll()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!configured_) {
    return Result::failure(hdi::DriverError::kInvalidState, "ROS topic driver is not configured");
  }
  const auto result = publishHoldLocked();
  if (result) {
    stopped_ = true;
  }
  return result;
}

hdi::DriverHealth RosTopicRobotDriver::health()
{
  std::lock_guard<std::mutex> lock(mutex_);
  hdi::DriverHealth result;
  result.connected = connected_;
  result.active = active_;
  result.details["state_topic"] = state_topic_;
  result.details["command_topic"] = command_topic_;
  result.details["configured_joint_count"] = std::to_string(mappings_.size());

  if (!configured_) {
    result.level = hdi::HealthLevel::kStale;
    result.message = "ROS topic driver is not configured";
    return result;
  }
  const auto now = Clock::now();
  if (!have_state_) {
    if (now - configured_at_ < startup_grace_) {
      result.level = hdi::HealthLevel::kWarning;
      result.communication_ok = true;
      result.message = "awaiting first ROS joint-state sample";
    } else {
      result.level = hdi::HealthLevel::kStale;
      result.message = "no valid ROS joint-state sample received";
    }
    return result;
  }
  if (!stateFreshLocked(now)) {
    result.level = hdi::HealthLevel::kStale;
    result.message = "ROS joint-state sample is stale";
    return result;
  }
  result.communication_ok = true;
  if (!active_) {
    result.level = hdi::HealthLevel::kWarning;
    result.message = "ROS topic driver is inactive";
  } else if (stopped_) {
    result.level = hdi::HealthLevel::kWarning;
    result.message = "ROS topic driver is holding position";
  } else {
    result.level = hdi::HealthLevel::kOk;
    result.message = "ROS topic driver operational";
  }
  return result;
}

void RosTopicRobotDriver::stateCallback(const sensor_msgs::msg::JointState::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!configured_) {
    return;
  }
  if (message->name.size() != message->position.size() ||
    (!message->velocity.empty() && message->velocity.size() != message->name.size()) ||
    (!message->effort.empty() && message->effort.size() != message->name.size()) ||
    !allFinite(message->position) || !allFinite(message->velocity) || !allFinite(message->effort))
  {
    last_feedback_error_ = "vendor JointState has inconsistent lengths or non-finite values";
    return;
  }

  std::unordered_map<std::string, std::size_t> vendor_indices;
  for (std::size_t index = 0; index < message->name.size(); ++index) {
    if (mapping_by_vendor_name_.count(message->name[index]) != 0U &&
      !vendor_indices.emplace(message->name[index], index).second)
    {
      last_feedback_error_ = "vendor JointState repeats a configured joint name";
      return;
    }
  }

  hdi::JointState next_state;
  next_state.joint_names.reserve(mappings_.size());
  next_state.positions.reserve(mappings_.size());
  if (!message->velocity.empty()) {
    next_state.velocities.reserve(mappings_.size());
  }
  if (!message->effort.empty()) {
    next_state.efforts.reserve(mappings_.size());
  }
  for (const auto & mapping : mappings_) {
    const auto found = vendor_indices.find(mapping.vendor_name);
    if (found == vendor_indices.end()) {
      last_feedback_error_ = "vendor JointState is missing configured joint '" + mapping.vendor_name + "'";
      return;
    }
    const auto index = found->second;
    next_state.joint_names.push_back(mapping.logical_name);
    next_state.positions.push_back(
      mapping.vendor_to_logical_scale * message->position[index] +
      mapping.vendor_to_logical_offset_rad);
    if (!message->velocity.empty()) {
      next_state.velocities.push_back(mapping.vendor_to_logical_scale * message->velocity[index]);
    }
    if (!message->effort.empty()) {
      next_state.efforts.push_back(message->effort[index] / mapping.vendor_to_logical_scale);
    }
  }
  next_state.sample_time = Clock::now();
  latest_state_ = std::move(next_state);
  last_state_received_ = latest_state_.sample_time;
  last_feedback_error_.clear();
  have_state_ = true;
}

bool RosTopicRobotDriver::stateFreshLocked(const Clock::time_point now) const
{
  return have_state_ && now >= last_state_received_ && now - last_state_received_ <= state_timeout_;
}

RosTopicRobotDriver::Result RosTopicRobotDriver::publishCommandLocked(const hdi::JointCommand & command)
{
  const auto count = command.joint_names.size();
  if (count == 0U || command.positions.size() != count ||
    (!command.velocities.empty() && command.velocities.size() != count) ||
    (!command.accelerations.empty() && command.accelerations.size() != count) ||
    (!command.efforts.empty() && command.efforts.size() != count) ||
    !allFinite(command.positions) || !allFinite(command.velocities) ||
    !allFinite(command.accelerations) || !allFinite(command.efforts))
  {
    return Result::failure(
      hdi::DriverError::kRejectedCommand, "joint command lengths or numeric values are invalid");
  }

  sensor_msgs::msg::JointState vendor_command;
  vendor_command.header.stamp = node_->now();
  vendor_command.name.reserve(count);
  vendor_command.position.reserve(count);
  if (!command.velocities.empty()) {
    vendor_command.velocity.reserve(count);
  }
  if (!command.efforts.empty()) {
    vendor_command.effort.reserve(count);
  }
  std::unordered_set<std::string> logical_names;
  for (std::size_t index = 0; index < count; ++index) {
    const auto found = mapping_by_logical_name_.find(command.joint_names[index]);
    if (found == mapping_by_logical_name_.end() ||
      !logical_names.insert(command.joint_names[index]).second)
    {
      return Result::failure(
        hdi::DriverError::kRejectedCommand, "joint command contains an unknown or duplicate joint");
    }
    const auto & mapping = mappings_[found->second];
    vendor_command.name.push_back(mapping.vendor_name);
    vendor_command.position.push_back(
      (command.positions[index] - mapping.vendor_to_logical_offset_rad) /
      mapping.vendor_to_logical_scale);
    if (!command.velocities.empty()) {
      vendor_command.velocity.push_back(command.velocities[index] / mapping.vendor_to_logical_scale);
    }
    if (!command.efforts.empty()) {
      vendor_command.effort.push_back(command.efforts[index] * mapping.vendor_to_logical_scale);
    }
  }
  try {
    command_publisher_->publish(std::move(vendor_command));
  } catch (const std::exception & error) {
    return Result::failure(
      hdi::DriverError::kCommunication,
      "failed to publish vendor joint command: " + std::string(error.what()));
  }
  return Result::success();
}

RosTopicRobotDriver::Result RosTopicRobotDriver::publishHoldLocked()
{
  if (!have_state_) {
    return Result::failure(
      hdi::DriverError::kNoFeedback, "cannot hold position before first valid joint state");
  }
  hdi::JointCommand hold;
  hold.joint_names = latest_state_.joint_names;
  hold.positions = latest_state_.positions;
  hold.velocities.assign(hold.positions.size(), 0.0);
  return publishCommandLocked(hold);
}

}  // namespace humanoid_driver_runtime

PLUGINLIB_EXPORT_CLASS(
  humanoid_driver_runtime::RosTopicRobotDriver,
  humanoid_driver_interface::RobotDriverPlugin)
