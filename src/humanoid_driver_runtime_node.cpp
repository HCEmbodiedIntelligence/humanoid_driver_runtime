// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "humanoid_driver_interface/types.hpp"
#include "humanoid_driver_runtime/diagnostics_runtime.hpp"
#include "humanoid_driver_runtime/driver_runtime.hpp"
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
    config.command_watchdog = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double, std::milli>(command_watchdog_ms));
    config.ros_node = this;
    for (std::size_t index = 0; index < joint_names.size(); ++index) {
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
    state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      state_topic, rclcpp::QoS(10).reliable());
    command_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      command_topic, rclcpp::QoS(10).reliable(),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        commandCallback(*message);
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
      feedback_ready_ = false;
      if (feedback.message != last_read_error_) {
        RCLCPP_WARN(get_logger(), "driver feedback unavailable: %s", feedback.message.c_str());
        last_read_error_ = feedback.message;
      }
      runtime_->enforceWatchdog(std::chrono::steady_clock::now());
      return;
    }
    feedback_ready_ = true;
    if (!last_read_error_.empty()) {
      RCLCPP_INFO(get_logger(), "driver feedback is active");
      last_read_error_.clear();
    }

    sensor_msgs::msg::JointState state;
    state.header.stamp = now();
    state.name = feedback.state.joint_names;
    state.position = feedback.state.positions;
    state.velocity = feedback.state.velocities;
    if (state.velocity.empty()) {
      state.velocity.assign(state.name.size(), 0.0);
    }
    state.effort = feedback.state.efforts;
    state_publisher_->publish(std::move(state));
    runtime_->enforceWatchdog(std::chrono::steady_clock::now());
  }

  void commandCallback(const sensor_msgs::msg::JointState & message)
  {
    if (!feedback_ready_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ignoring platform joint command until valid robot feedback is available");
      return;
    }
    hdi::JointCommand command;
    command.joint_names = message.name;
    command.positions = message.position;
    command.velocities = message.velocity;
    command.efforts = message.effort;
    const double first_position =
      command.positions.empty() ? 0.0 : command.positions.front();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "TRACE driver IN /hc_teleop/joint_cmd joints=%zu first_position=%.4f",
      command.joint_names.size(), first_position);
    std::string error;
    if (!runtime_->write(command, error)) {
      RCLCPP_ERROR(get_logger(), "platform joint command rejected: %s", error.c_str());
      return;
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "TRACE driver OUT /openarmx/vendor/joint_command joints=%zu first_position=%.4f",
      command.joint_names.size(), first_position);
  }

  void publishDiagnostics()
  {
    diagnostic_publisher_->publish(
      diagnostics_.makeDriverReport(
        runtime_->status(), now(), get_fully_qualified_name(), plugin_class_));
  }

  std::string plugin_class_;
  std::unique_ptr<DriverRuntime> runtime_;
  DiagnosticsRuntime diagnostics_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_subscription_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  std::string last_read_error_;
  bool feedback_ready_{false};
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
