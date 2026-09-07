// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "humanoid_driver_interface/types.hpp"
#include "humanoid_driver_runtime/diagnostics_runtime.hpp"
#include "humanoid_driver_runtime/gripper_runtime.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace hdi = humanoid_driver_interface;

namespace humanoid_driver_runtime
{
namespace
{

bool finite(const std::vector<double> & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {return std::isfinite(value);});
}

class HumanoidGripperRuntimeNode final : public rclcpp::Node
{
public:
  HumanoidGripperRuntimeNode()
  : Node("humanoid_gripper_runtime")
  {
    plugin_class_ = declare_parameter<std::string>("plugin_class", "");
    const auto plugin_xml_paths = declare_parameter<std::vector<std::string>>(
      "plugin_xml_paths", std::vector<std::string>{});
    const auto names = declare_parameter<std::vector<std::string>>(
      "gripper_names", std::vector<std::string>{});
    const auto vendor_names = declare_parameter<std::vector<std::string>>(
      "vendor_gripper_names", std::vector<std::string>{});
    const auto units = declare_parameter<std::vector<std::string>>(
      "position_units", std::vector<std::string>{});
    auto scales = declare_parameter<std::vector<double>>(
      "vendor_to_logical_scales", std::vector<double>{});
    auto offsets = declare_parameter<std::vector<double>>(
      "vendor_to_logical_offsets", std::vector<double>{});
    const auto plugin_parameters = declare_parameter<std::vector<std::string>>(
      "plugin_parameters", std::vector<std::string>{});
    const auto state_topic = declare_parameter<std::string>(
      "platform_gripper_state_topic", "/hc_teleop/gripper_states");
    const auto command_topic = declare_parameter<std::string>(
      "platform_gripper_command_topic", "/hc_teleop/gripper_commands");
    const auto diagnostics_topic = declare_parameter<std::string>(
      "diagnostics_topic", "/diagnostics");
    const auto control_frequency = declare_parameter<double>("control_frequency_hz", 50.0);
    const auto diagnostic_frequency = declare_parameter<double>("diagnostic_frequency_hz", 10.0);
    const auto watchdog_ms = declare_parameter<double>("command_watchdog_ms", 250.0);

    if (plugin_class_.empty() || names.empty() || names.size() != vendor_names.size() ||
      names.size() != units.size() || state_topic.empty() || command_topic.empty() ||
      state_topic == command_topic || diagnostics_topic.empty() ||
      !std::isfinite(control_frequency) || control_frequency <= 0.0 ||
      !std::isfinite(diagnostic_frequency) || diagnostic_frequency <= 0.0 ||
      !std::isfinite(watchdog_ms) || watchdog_ms <= 0.0)
    {
      throw std::invalid_argument("gripper runtime parameters are incomplete or invalid");
    }
    if (scales.empty()) {
      scales.assign(names.size(), 1.0);
    }
    if (offsets.empty()) {
      offsets.assign(names.size(), 0.0);
    }
    if (scales.size() != names.size() || offsets.size() != names.size() ||
      !finite(scales) || !finite(offsets))
    {
      throw std::invalid_argument("gripper conversion parameters are invalid");
    }

    GripperRuntimeConfig config;
    config.plugin_class = plugin_class_;
    config.plugin_xml_paths = plugin_xml_paths;
    config.ros_node = this;
    config.command_watchdog = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double, std::milli>(watchdog_ms));
    for (std::size_t index = 0; index < names.size(); ++index) {
      config.plugin_configuration.grippers.push_back(
        {names[index], vendor_names[index], units[index], scales[index], offsets[index]});
    }
    for (const auto & entry : plugin_parameters) {
      const auto separator = entry.find('=');
      if (separator == std::string::npos || separator == 0U || separator + 1U >= entry.size()) {
        throw std::invalid_argument("gripper plugin parameters must use non-empty key=value syntax");
      }
      const auto key = entry.substr(0, separator);
      if (!config.plugin_configuration.parameters.emplace(
          key, entry.substr(separator + 1U)).second)
      {
        throw std::invalid_argument("duplicate gripper plugin parameter '" + key + "'");
      }
    }

    runtime_ = std::make_unique<GripperRuntime>(config);
    state_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      state_topic, rclcpp::QoS(10).reliable());
    command_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      command_topic, rclcpp::QoS(10).reliable(),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {command(*message);});
    diagnostic_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic, rclcpp::QoS(10).reliable());
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / control_frequency)),
      std::bind(&HumanoidGripperRuntimeNode::controlTick, this));
    diagnostic_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / diagnostic_frequency)),
      std::bind(&HumanoidGripperRuntimeNode::diagnosticsTick, this));
  }

private:
  void controlTick()
  {
    const auto feedback = runtime_->read();
    if (feedback.successful) {
      feedback_ready_ = true;
      sensor_msgs::msg::JointState message;
      message.header.stamp = now();
      message.name = feedback.state.gripper_names;
      message.position = feedback.state.positions;
      message.effort = feedback.state.efforts;
      state_publisher_->publish(std::move(message));
    } else {
      feedback_ready_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "gripper feedback unavailable: %s",
        feedback.message.c_str());
    }
    runtime_->enforceWatchdog(std::chrono::steady_clock::now());
  }

  void command(const sensor_msgs::msg::JointState & message)
  {
    if (!feedback_ready_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ignoring gripper command until valid measured feedback is available");
      return;
    }
    hdi::GripperCommand command;
    command.gripper_names = message.name;
    command.positions = message.position;
    command.max_efforts = message.effort;
    std::string error;
    if (!runtime_->write(command, error)) {
      RCLCPP_ERROR(get_logger(), "platform gripper command rejected: %s", error.c_str());
    }
  }

  void diagnosticsTick()
  {
    diagnostic_publisher_->publish(
      diagnostics_.makeDriverReport(runtime_->status(), now(), get_fully_qualified_name(),
        plugin_class_));
  }

  std::string plugin_class_;
  std::unique_ptr<GripperRuntime> runtime_;
  DiagnosticsRuntime diagnostics_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_subscription_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  bool feedback_ready_{false};
};

}  // namespace
}  // namespace humanoid_driver_runtime

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<humanoid_driver_runtime::HumanoidGripperRuntimeNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("humanoid_gripper_runtime"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
