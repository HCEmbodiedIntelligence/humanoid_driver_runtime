// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "humanoid_driver_interface/types.hpp"
#include "humanoid_driver_runtime/activity_log.hpp"
#include "humanoid_driver_runtime/diagnostics_runtime.hpp"
#include "humanoid_driver_runtime/driver_runtime.hpp"
#include "humanoid_driver_runtime/sample_timestamp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace hdi = humanoid_driver_interface;

namespace humanoid_driver_runtime
{
namespace
{

bool allFinite(const std::vector<double> & values)
{
  return std::all_of(
    values.begin(), values.end(), [](const double value) {return std::isfinite(value);});
}

class HumanoidDriverRuntimeNode final : public rclcpp::Node
{
public:
  HumanoidDriverRuntimeNode()
  : Node("humanoid_driver_runtime")
  {
    plugin_class_ = declare_parameter<std::string>("plugin_class", "");
    const auto plugin_xml_paths = declare_parameter<std::vector<std::string>>(
      "plugin_xml_paths", std::vector<std::string>{});
    const auto joint_names = declare_parameter<std::vector<std::string>>(
      "joint_names", std::vector<std::string>{});
    const auto vendor_joint_names = declare_parameter<std::vector<std::string>>(
      "vendor_joint_names", std::vector<std::string>{});
    const auto vendor_joint_groups = declare_parameter<std::vector<std::string>>(
      "vendor_joint_groups", std::vector<std::string>{});
    auto vendor_to_logical_scales = declare_parameter<std::vector<double>>(
      "vendor_to_logical_scales", std::vector<double>{});
    auto vendor_to_logical_offsets = declare_parameter<std::vector<double>>(
      "vendor_to_logical_offsets_rad", std::vector<double>{});
    const auto plugin_parameters = declare_parameter<std::vector<std::string>>(
      "plugin_parameters", std::vector<std::string>{});

    const auto state_topic = declare_parameter<std::string>(
      "platform_joint_state_topic", "/hc_teleop/joint_states");
    const auto command_topic = declare_parameter<std::string>(
      "platform_joint_command_topic", "/hc_teleop/joint_cmd");
    const auto diagnostics_topic = declare_parameter<std::string>(
      "diagnostics_topic", "/diagnostics");
    const auto control_frequency_hz = declare_parameter<double>(
      "control_frequency_hz", 100.0);
    const auto diagnostic_frequency_hz = declare_parameter<double>(
      "diagnostic_frequency_hz", 10.0);
    command_max_age_ms_ = declare_parameter<int>("command_max_age_ms", 100);
    const auto feedback_max_age_ms = declare_parameter<int>("feedback_max_age_ms", 100);
    if (command_max_age_ms_ <= 0 || feedback_max_age_ms <= 0) {
      throw std::invalid_argument("command/feedback maximum ages must be positive");
    }
    const auto command_watchdog_ms = declare_parameter<double>(
      "command_watchdog_ms", 100.0);

    if (plugin_class_.empty() || state_topic.empty() || command_topic.empty() ||
      diagnostics_topic.empty() || joint_names.empty() ||
      joint_names.size() != vendor_joint_names.size() ||
      joint_names.size() != vendor_joint_groups.size() ||
      !std::isfinite(control_frequency_hz) || control_frequency_hz <= 0.0 ||
      control_frequency_hz > 1000.0 || !std::isfinite(diagnostic_frequency_hz) ||
      diagnostic_frequency_hz <= 0.0 || !std::isfinite(command_watchdog_ms) ||
      command_watchdog_ms <= 0.0)
    {
      throw std::invalid_argument("driver runtime parameters are incomplete or invalid");
    }
    if (vendor_to_logical_scales.empty()) {
      vendor_to_logical_scales.assign(joint_names.size(), 1.0);
    }
    if (vendor_to_logical_offsets.empty()) {
      vendor_to_logical_offsets.assign(joint_names.size(), 0.0);
    }
    if (vendor_to_logical_scales.size() != joint_names.size() ||
      vendor_to_logical_offsets.size() != joint_names.size() ||
      !allFinite(vendor_to_logical_scales) || !allFinite(vendor_to_logical_offsets) ||
      std::any_of(
        vendor_to_logical_scales.begin(), vendor_to_logical_scales.end(),
        [](const double scale) {return std::abs(scale) < 1.0e-12;}))
    {
      throw std::invalid_argument("driver joint conversion parameters are invalid");
    }

    DriverRuntimeConfig config;
    config.plugin_class = plugin_class_;
    config.plugin_xml_paths = plugin_xml_paths;
    config.command_watchdog = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double, std::milli>(command_watchdog_ms));
    config.ros_node = this;
    config.feedback_max_age = std::chrono::milliseconds(feedback_max_age_ms);
    for (std::size_t index = 0; index < joint_names.size(); ++index) {
      joint_groups_[joint_names[index]] = vendor_joint_groups[index];
      command_timestamps_.emplace(joint_names[index], SampleTimestamp{});
      config.plugin_configuration.joints.push_back(
        {joint_names[index], vendor_joint_names[index], vendor_joint_groups[index],
          vendor_to_logical_scales[index], vendor_to_logical_offsets[index]});
    }
    for (const auto & entry : plugin_parameters) {
      const auto separator = entry.find('=');
      if (separator == std::string::npos || separator == 0U || separator + 1U >= entry.size()) {
        throw std::invalid_argument(
                "plugin parameter '" + entry + "' must use non-empty key=value syntax");
      }
      const auto key = entry.substr(0, separator);
      if (!config.plugin_configuration.parameters.emplace(
          key, entry.substr(separator + 1U)).second)
      {
        throw std::invalid_argument("duplicate plugin parameter '" + key + "'");
      }
    }

    runtime_ = std::make_unique<DriverRuntime>(config);
    activity_log_ = std::make_unique<JointActivityLog>(
      joint_names, vendor_joint_groups, std::chrono::duration<double, std::milli>(command_watchdog_ms));
    state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      state_topic, rclcpp::QoS(10).reliable());
    command_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      command_topic, rclcpp::QoS(10).reliable(),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        commandCallback(*message);
        // Drain the bounded DDS history before the next control tick. Retain
        // newest values per joint, so one arm never overwrites the other arm.
        sensor_msgs::msg::JointState queued;
        rclcpp::MessageInfo info;
        for (int i = 0; i < 32 && command_subscription_->take(queued, info); ++i) {
          commandCallback(queued);
        }
      });
    diagnostic_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic, rclcpp::QoS(10).reliable());

    const auto control_period = std::chrono::duration<double>(1.0 / control_frequency_hz);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(control_period),
      std::bind(&HumanoidDriverRuntimeNode::controlTick, this));
    const auto diagnostic_period = std::chrono::duration<double>(1.0 / diagnostic_frequency_hz);
    diagnostic_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(diagnostic_period),
      std::bind(&HumanoidDriverRuntimeNode::publishDiagnostics, this));

    RCLCPP_INFO(
      get_logger(), "driver runtime ready: plugin=%s, state=%s, command=%s, joints=%zu",
      plugin_class_.c_str(), state_topic.c_str(), command_topic.c_str(), joint_names.size());
  }

private:
  void controlTick()
  {
    const auto feedback = runtime_->read();
    if (!feedback.successful) {
      activity_log_->unavailable();
      feedback_ready_ = false;
      pending_commands_.clear();
      runtime_->enforceWatchdog(std::chrono::steady_clock::now());
      return;
    }
    feedback_ready_ = true;
    activity_log_->sample(
      feedback.state.joint_names, feedback.state.positions, std::chrono::steady_clock::now());

    const auto clock_now = std::chrono::steady_clock::now();
    if (!last_published_sample_ || feedback.state.sample_time != *last_published_sample_) {
      sensor_msgs::msg::JointState state;
      const auto age = std::chrono::duration_cast<std::chrono::nanoseconds>(
        clock_now - feedback.state.sample_time);
      const auto ros_now = now();
      // A simulated clock may not have started yet. Do not emit zero or
      // negative feedback timestamps or throw while subtracting sample age.
      if (ros_now.nanoseconds() <= age.count()) {
        pending_commands_.clear();
        feedback_ready_ = false;
        runtime_->enforceWatchdog(clock_now);
        return;
      }
      state.header.stamp = ros_now - rclcpp::Duration(age);
      state.name = feedback.state.joint_names;
      state.position = feedback.state.positions;
      state.velocity = feedback.state.velocities;
      if (state.velocity.empty()) {
        state.velocity.assign(state.name.size(), 0.0);
      }
      state.effort = feedback.state.efforts;
      state_publisher_->publish(std::move(state));
      last_published_sample_ = feedback.state.sample_time;
    }
    runtime_->enforceWatchdog(clock_now);
    flushCommands();
  }

  struct PendingJoint
  {
    double position;
    std::optional<double> velocity;
    std::optional<double> effort;
    std::chrono::steady_clock::time_point valid_until;
  };

  void commandCallback(const sensor_msgs::msg::JointState & message)
  {
    const auto count = message.name.size();
    if (!feedback_ready_ || count == 0 || message.position.size() != count ||
      (!message.velocity.empty() && message.velocity.size() != count) ||
      (!message.effort.empty() && message.effort.size() != count) ||
      !allFinite(message.position) || !allFinite(message.velocity) || !allFinite(message.effort) ||
      message.header.stamp.sec < 0 || message.header.stamp.nanosec >= 1000000000U)
    {
      return;
    }
    const auto received = std::chrono::steady_clock::now();
    const auto ros_now = now().nanoseconds();
    const std::int64_t stamp = static_cast<std::int64_t>(message.header.stamp.sec) *
      1000000000LL + message.header.stamp.nanosec;
    std::set<std::string> names;
    auto updated = command_timestamps_;
    auto sampled = received;
    for (const auto & name : message.name) {
      const auto found = updated.find(name);
      if (!names.insert(name).second || found == updated.end() ||
        !found->second.accept(stamp, ros_now, received,
          std::chrono::milliseconds(command_max_age_ms_), sampled))
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
          "discarding stale, duplicate, out-of-order or malformed joint command");
        return;
      }
    }
    command_timestamps_ = std::move(updated);
    for (std::size_t i = 0; i < count; ++i) {
      pending_commands_[message.name[i]] = {
        message.position[i],
        message.velocity.empty() ? std::nullopt : std::optional<double>(message.velocity[i]),
        message.effort.empty() ? std::nullopt : std::optional<double>(message.effort[i]),
        sampled + std::chrono::milliseconds(command_max_age_ms_)};
    }
  }

  void flushCommands()
  {
    std::map<std::string, hdi::JointCommand> groups;
    std::map<std::string, std::chrono::steady_clock::time_point> deadlines;
    std::map<std::string, bool> velocities, efforts;
    const auto clock_now = std::chrono::steady_clock::now();
    for (const auto & [joint, pending] : pending_commands_) {
      if (clock_now >= pending.valid_until) {
        continue;
      }
      const auto & group = joint_groups_.at(joint);
      auto & command = groups[group];
      if (command.joint_names.empty()) {
        deadlines[group] = pending.valid_until;
      }
      deadlines[group] = std::min(deadlines[group], pending.valid_until);
      command.joint_names.push_back(joint);
      command.positions.push_back(pending.position);
      velocities[group] = velocities[group] || pending.velocity.has_value();
      efforts[group] = efforts[group] || pending.effort.has_value();
      command.velocities.push_back(pending.velocity.value_or(0.0));
      command.efforts.push_back(pending.effort.value_or(0.0));
    }
    pending_commands_.clear();
    for (auto & [group, command] : groups) {
      if (!velocities[group]) {command.velocities.clear();}
      if (!efforts[group]) {command.efforts.clear();}
      std::string error;
      if (!runtime_->write(command, error, deadlines.at(group))) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000,
          "platform joint command rejected: %s", error.c_str());
        continue;
      }
      activity_log_->acceptedCommand(command.joint_names, std::chrono::steady_clock::now());
    }
  }

  void publishDiagnostics()
  {
    const auto status = runtime_->status();
    const auto report = diagnostics_.makeDriverReport(
      status, now(), get_fully_qualified_name(), plugin_class_);
    const auto clock_now = std::chrono::steady_clock::now();
    for (const auto & message : activity_log_->poll(clock_now)) {
      RCLCPP_INFO(get_logger(), "%s", message.c_str());
    }
    const bool pending = status.feedback_waiting || status.driver_fault_latched ||
      status.feedback_recovery_count != logged_feedback_recoveries_;
    if (pending && (!have_feedback_log_ || clock_now - last_feedback_log_ >= std::chrono::seconds(30))) {
      const auto & message = report.status.front().message;
      const auto interruptions = static_cast<unsigned long long>(status.feedback_interruption_count);
      const auto recoveries = static_cast<unsigned long long>(status.feedback_recovery_count);
      if (status.driver_fault_latched) {
        RCLCPP_ERROR(
          get_logger(), "driver state: %s; feedback_interruptions=%llu feedback_recoveries=%llu",
          message.c_str(), interruptions, recoveries);
      } else if (status.feedback_waiting) {
        RCLCPP_WARN(
          get_logger(), "driver state: %s; feedback_interruptions=%llu feedback_recoveries=%llu; cause=%s",
          message.c_str(), interruptions, recoveries, status.last_stop_reason.c_str());
      } else {
        RCLCPP_INFO(
          get_logger(), "driver feedback recovered; feedback_interruptions=%llu feedback_recoveries=%llu",
          interruptions, recoveries);
      }
      have_feedback_log_ = true;
      last_feedback_log_ = clock_now;
      logged_feedback_recoveries_ = status.feedback_recovery_count;
    }
    diagnostic_publisher_->publish(
      report);
  }

  std::string plugin_class_;
  std::unique_ptr<DriverRuntime> runtime_;
  std::unique_ptr<JointActivityLog> activity_log_;
  DiagnosticsRuntime diagnostics_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_subscription_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  bool have_feedback_log_{false};
  std::chrono::steady_clock::time_point last_feedback_log_;
  std::uint64_t logged_feedback_recoveries_{0};
  bool feedback_ready_{false};
  int command_max_age_ms_{100};
  std::map<std::string, SampleTimestamp> command_timestamps_;
  std::map<std::string, std::string> joint_groups_;
  std::map<std::string, PendingJoint> pending_commands_;
  std::optional<std::chrono::steady_clock::time_point> last_published_sample_;
};

}  // namespace
}  // namespace humanoid_driver_runtime

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<humanoid_driver_runtime::HumanoidDriverRuntimeNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("humanoid_driver_runtime"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
