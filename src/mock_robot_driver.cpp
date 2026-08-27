// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "humanoid_driver_runtime/mock_robot_driver.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include "pluginlib/class_list_macros.hpp"

namespace hdi = humanoid_driver_interface;

namespace
{

bool parseBool(const std::string & value)
{
  if (value == "true" || value == "1" || value == "yes") {
    return true;
  }
  if (value == "false" || value == "0" || value == "no") {
    return false;
  }
  throw std::invalid_argument("expected true/false, got '" + value + "'");
}

double parseFiniteDouble(const std::string & key, const std::string & value)
{
  std::size_t consumed = 0;
  const double parsed = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(parsed)) {
    throw std::invalid_argument(key + " must be a finite number");
  }
  return parsed;
}

}  // namespace

namespace humanoid_driver_runtime
{

hdi::DriverResult MockRobotDriver::configure(const hdi::DriverConfiguration & configuration)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (connected_ || active_) {
    return hdi::DriverResult::failure(
      hdi::DriverError::kInvalidState, "cannot configure a connected mock driver");
  }
  if (configuration.joints.empty()) {
    return hdi::DriverResult::failure(
      hdi::DriverError::kInvalidConfiguration, "at least one joint mapping is required");
  }

  std::unordered_set<std::string> logical_names;
  std::unordered_set<std::string> vendor_keys;
  for (const auto & joint : configuration.joints) {
    if (joint.logical_name.empty() || joint.vendor_name.empty() || joint.vendor_group.empty()) {
      return hdi::DriverResult::failure(
        hdi::DriverError::kInvalidConfiguration, "joint mapping fields must not be empty");
    }
    if (!logical_names.insert(joint.logical_name).second) {
      return hdi::DriverResult::failure(
        hdi::DriverError::kInvalidConfiguration,
        "duplicate logical joint '" + joint.logical_name + "'");
    }
    const auto vendor_key = joint.vendor_group + "\n" + joint.vendor_name;
    if (!vendor_keys.insert(vendor_key).second) {
      return hdi::DriverResult::failure(
        hdi::DriverError::kInvalidConfiguration,
        "duplicate vendor joint '" + joint.vendor_group + "/" + joint.vendor_name + "'");
    }
  }

  velocity_limit_rad_s_ = std::numeric_limits<double>::infinity();
  latency_seconds_ = 0.0;
  position_error_rad_ = 0.0;
  initial_position_rad_ = 0.0;
  inject_disconnect_ = false;
  inject_write_failure_ = false;
  try {
    for (const auto & [key, value] : configuration.parameters) {
      if (key == "velocity_limit_rad_s") {
        velocity_limit_rad_s_ = parseFiniteDouble(key, value);
        if (velocity_limit_rad_s_ <= 0.0) {
          throw std::invalid_argument(key + " must be greater than zero");
        }
      } else if (key == "latency_s") {
        latency_seconds_ = parseFiniteDouble(key, value);
        if (latency_seconds_ < 0.0) {
          throw std::invalid_argument(key + " must not be negative");
        }
      } else if (key == "position_error_rad") {
        position_error_rad_ = parseFiniteDouble(key, value);
      } else if (key == "initial_position_rad") {
        initial_position_rad_ = parseFiniteDouble(key, value);
      } else if (key == "inject_disconnect") {
        inject_disconnect_ = parseBool(value);
      } else if (key == "inject_write_failure") {
        inject_write_failure_ = parseBool(value);
      } else {
        throw std::invalid_argument("unknown mock parameter '" + key + "'");
      }
    }
  } catch (const std::exception & exception) {
    return hdi::DriverResult::failure(hdi::DriverError::kInvalidConfiguration, exception.what());
  }

  configuration_ = configuration;
  joint_indices_.clear();
  for (std::size_t index = 0; index < configuration_.joints.size(); ++index) {
    joint_indices_.emplace(configuration_.joints[index].logical_name, index);
  }
  positions_.assign(configuration_.joints.size(), initial_position_rad_);
  velocities_.assign(configuration_.joints.size(), 0.0);
  target_positions_ = positions_;
  pending_targets_.clear();
  last_update_ = std::chrono::steady_clock::now();
  configured_ = true;
  streaming_ = false;
  stopped_ = true;
  stop_count_ = 0;
  write_count_ = 0;
  return hdi::DriverResult::success("mock configured");
}

hdi::DriverResult MockRobotDriver::connect()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!configured_ || active_) {
    return hdi::DriverResult::failure(
      hdi::DriverError::kInvalidState, "mock must be configured and inactive before connect");
  }
  if (inject_disconnect_) {
    return hdi::DriverResult::failure(
      hdi::DriverError::kCommunication, "injected disconnect during connect");
  }
  connected_ = true;
  last_update_ = std::chrono::steady_clock::now();
  return hdi::DriverResult::success("mock connected");
}

hdi::DriverResult MockRobotDriver::disconnect()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_) {
    return hdi::DriverResult::failure(
      hdi::DriverError::kInvalidState, "deactivate mock before disconnect");
  }
  connected_ = false;
  streaming_ = false;
  stopped_ = true;
  pending_targets_.clear();
  return hdi::DriverResult::success("mock disconnected");
}

hdi::DriverResult MockRobotDriver::activate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!connected_ || active_) {
    return hdi::DriverResult::failure(
      hdi::DriverError::kInvalidState, "mock must be connected and inactive before activate");
  }
  if (!communicationAvailableLocked()) {
    return hdi::DriverResult::failure(hdi::DriverError::kCommunication, "injected disconnect");
  }
  active_ = true;
  stopped_ = true;
  last_update_ = std::chrono::steady_clock::now();
  return hdi::DriverResult::success("mock active");
}

hdi::DriverResult MockRobotDriver::deactivate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_) {
    return hdi::DriverResult::success("mock already inactive");
  }
  advanceLocked(std::chrono::steady_clock::now());
  target_positions_ = positions_;
  velocities_.assign(velocities_.size(), 0.0);
  pending_targets_.clear();
  streaming_ = false;
  stopped_ = true;
  active_ = false;
  return hdi::DriverResult::success("mock inactive");
}

hdi::DriverResult MockRobotDriver::readJointState(hdi::JointState & state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || !streaming_) {
    return hdi::DriverResult::failure(
      hdi::DriverError::kNotActive, "mock joint stream is not active");
  }
  if (!communicationAvailableLocked()) {
    return hdi::DriverResult::failure(hdi::DriverError::kCommunication, "injected disconnect");
  }
  const auto now = std::chrono::steady_clock::now();
  advanceLocked(now);
  state.joint_names.clear();
  state.joint_names.reserve(configuration_.joints.size());
  for (const auto & joint : configuration_.joints) {
    state.joint_names.push_back(joint.logical_name);
  }
  state.positions = positions_;
  for (auto & position : state.positions) {
    position += position_error_rad_;
  }
  state.velocities = velocities_;
  state.efforts.assign(configuration_.joints.size(), 0.0);
  state.sample_time = now;
  return hdi::DriverResult::success();
}

hdi::DriverResult MockRobotDriver::startJointStream()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_) {
    return hdi::DriverResult::failure(hdi::DriverError::kNotActive, "mock is not active");
  }
  if (!communicationAvailableLocked()) {
    return hdi::DriverResult::failure(hdi::DriverError::kCommunication, "injected disconnect");
  }
  streaming_ = true;
  last_update_ = std::chrono::steady_clock::now();
  return hdi::DriverResult::success("mock joint stream started");
}

hdi::DriverResult MockRobotDriver::writeJointCommand(const hdi::JointCommand & command)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || !streaming_) {
    return hdi::DriverResult::failure(
      hdi::DriverError::kNotActive, "mock joint stream is not active");
  }
  if (!communicationAvailableLocked()) {
    return hdi::DriverResult::failure(hdi::DriverError::kCommunication, "injected disconnect");
  }
  if (inject_write_failure_) {
    return hdi::DriverResult::failure(
      hdi::DriverError::kCommunication, "injected command write failure");
  }
  if (command.joint_names.empty() || command.positions.size() != command.joint_names.size()) {
    return hdi::DriverResult::failure(
      hdi::DriverError::kRejectedCommand, "mock command requires one position per joint");
  }

  const auto now = std::chrono::steady_clock::now();
  advanceLocked(now);
  std::vector<double> next_target =
    pending_targets_.empty() ? target_positions_ : pending_targets_.back().positions;
  std::unordered_set<std::string> command_names;
  for (std::size_t index = 0; index < command.joint_names.size(); ++index) {
    const auto position = command.positions[index];
    const auto joint = joint_indices_.find(command.joint_names[index]);
    if (joint == joint_indices_.end() || !command_names.insert(command.joint_names[index]).second ||
      !std::isfinite(position))
    {
      return hdi::DriverResult::failure(
        hdi::DriverError::kRejectedCommand, "mock received an invalid joint command");
    }
    next_target[joint->second] = position;
  }
  pending_targets_.push_back(
    PendingTarget{now + std::chrono::duration_cast<TimePoint::duration>(
        std::chrono::duration<double>(latency_seconds_)), std::move(next_target)});
  stopped_ = false;
  ++write_count_;
  return hdi::DriverResult::success();
}

hdi::DriverResult MockRobotDriver::stopJointStream()
{
  std::lock_guard<std::mutex> lock(mutex_);
  advanceLocked(std::chrono::steady_clock::now());
  target_positions_ = positions_;
  pending_targets_.clear();
  velocities_.assign(velocities_.size(), 0.0);
  streaming_ = false;
  stopped_ = true;
  return hdi::DriverResult::success("mock joint stream stopped");
}

hdi::DriverResult MockRobotDriver::stopAll()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (configured_) {
    advanceLocked(std::chrono::steady_clock::now());
    target_positions_ = positions_;
    pending_targets_.clear();
    velocities_.assign(velocities_.size(), 0.0);
  }
  stopped_ = true;
  ++stop_count_;
  return hdi::DriverResult::success("mock holding current position");
}

hdi::DriverHealth MockRobotDriver::health()
{
  std::lock_guard<std::mutex> lock(mutex_);
  hdi::DriverHealth result;
  result.connected = connected_;
  result.active = active_;
  result.communication_ok = communicationAvailableLocked();
  if (!configured_) {
    result.level = hdi::HealthLevel::kStale;
    result.message = "mock not configured";
  } else if (!result.communication_ok) {
    result.level = hdi::HealthLevel::kError;
    result.message = "mock communication disconnected";
  } else if (stopped_) {
    result.level = hdi::HealthLevel::kWarning;
    result.message = "mock holding position";
  } else {
    result.level = hdi::HealthLevel::kOk;
    result.message = "mock operational";
  }
  result.details["streaming"] = streaming_ ? "true" : "false";
  result.details["writes"] = std::to_string(write_count_);
  result.details["stops"] = std::to_string(stop_count_);
  return result;
}

void MockRobotDriver::setDisconnectInjected(bool enabled)
{
  std::lock_guard<std::mutex> lock(mutex_);
  inject_disconnect_ = enabled;
}

void MockRobotDriver::setWriteFailureInjected(bool enabled)
{
  std::lock_guard<std::mutex> lock(mutex_);
  inject_write_failure_ = enabled;
}

std::uint64_t MockRobotDriver::stopCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return stop_count_;
}

void MockRobotDriver::advanceLocked(TimePoint now)
{
  while (!pending_targets_.empty() && pending_targets_.front().apply_at <= now) {
    integrateLocked(pending_targets_.front().apply_at);
    target_positions_ = std::move(pending_targets_.front().positions);
    pending_targets_.pop_front();
  }
  integrateLocked(now);
}

void MockRobotDriver::integrateLocked(TimePoint end)
{
  const double elapsed = std::chrono::duration<double>(end - last_update_).count();
  if (elapsed < 0.0) {
    last_update_ = end;
    return;
  }
  for (std::size_t index = 0; index < positions_.size(); ++index) {
    const double difference = target_positions_[index] - positions_[index];
    if (std::isinf(velocity_limit_rad_s_)) {
      positions_[index] = target_positions_[index];
      velocities_[index] = 0.0;
      continue;
    }
    const double maximum_step = velocity_limit_rad_s_ * elapsed;
    const double step = std::clamp(difference, -maximum_step, maximum_step);
    positions_[index] += step;
    velocities_[index] = elapsed > 0.0 ? step / elapsed : 0.0;
    if (std::abs(target_positions_[index] - positions_[index]) < 1e-12) {
      positions_[index] = target_positions_[index];
      velocities_[index] = 0.0;
    }
  }
  last_update_ = end;
}

bool MockRobotDriver::communicationAvailableLocked() const
{
  return connected_ && !inject_disconnect_;
}

}  // namespace humanoid_driver_runtime

PLUGINLIB_EXPORT_CLASS(
  humanoid_driver_runtime::MockRobotDriver,
  humanoid_driver_interface::RobotDriverPlugin)
