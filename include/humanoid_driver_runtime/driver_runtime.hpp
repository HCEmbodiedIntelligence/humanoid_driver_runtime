// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#ifndef HUMANOID_DRIVER_RUNTIME__DRIVER_RUNTIME_HPP_
#define HUMANOID_DRIVER_RUNTIME__DRIVER_RUNTIME_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "humanoid_driver_interface/robot_driver_plugin.hpp"
#include "pluginlib/class_loader.hpp"

namespace rclcpp
{
class Node;
}  // namespace rclcpp

namespace humanoid_driver_runtime
{

struct DriverRuntimeConfig
{
  std::string plugin_class;
  humanoid_driver_interface::DriverConfiguration plugin_configuration;
  std::chrono::milliseconds command_watchdog{100};
  // Required only when plugin_class implements Ros2DriverPlugin. The runtime node owns this
  // pointer for the complete DriverRuntime lifetime.
  rclcpp::Node * ros_node{nullptr};
};

struct DriverReadResult
{
  bool successful{false};
  humanoid_driver_interface::JointState state;
  std::string message;
};

struct DriverRuntimeStatus
{
  humanoid_driver_interface::DriverHealth health;
  bool watchdog_stopped{false};
  bool driver_fault_latched{false};
  std::uint64_t watchdog_stop_count{0};
  std::uint64_t safety_stop_count{0};
  std::uint64_t rejected_command_count{0};
  std::string last_stop_reason;
};

/// Owns exactly one hardware plugin and exposes synchronous, ROS-independent
/// read/write/stop operations to the unified control node.
class DriverRuntime
{
public:
  explicit DriverRuntime(const DriverRuntimeConfig & config);
  ~DriverRuntime();

  DriverRuntime(const DriverRuntime &) = delete;
  DriverRuntime & operator=(const DriverRuntime &) = delete;

  DriverReadResult read();
  bool write(const humanoid_driver_interface::JointCommand & command, std::string & error);
  void enforceWatchdog(std::chrono::steady_clock::time_point now);
  void stop(const std::string & reason, bool driver_fault = false);
  DriverRuntimeStatus status();

  const std::vector<humanoid_driver_interface::JointMapping> & jointMappings() const noexcept;

private:
  using RobotDriverPlugin = humanoid_driver_interface::RobotDriverPlugin;
  using JointCommand = humanoid_driver_interface::JointCommand;
  using JointState = humanoid_driver_interface::JointState;
  using JointMapping = humanoid_driver_interface::JointMapping;

  static void validateConfiguration(const DriverRuntimeConfig & config);
  bool validateCommand(const JointCommand & command, std::string & reason) const;
  bool validateState(const JointState & state, std::string & reason) const;
  static bool allFinite(const std::vector<double> & values);
  void stopLocked(const std::string & reason, bool driver_fault);
  void shutdownLocked() noexcept;

  pluginlib::ClassLoader<RobotDriverPlugin> plugin_loader_;
  std::shared_ptr<RobotDriverPlugin> plugin_;
  std::vector<JointMapping> joint_mappings_;
  std::unordered_map<std::string, JointMapping> mapping_by_logical_name_;
  std::chrono::milliseconds watchdog_timeout_;
  std::chrono::steady_clock::time_point last_command_time_;

  mutable std::mutex mutex_;
  bool active_{false};
  bool watchdog_latched_{false};
  bool driver_fault_latched_{false};
  std::uint64_t watchdog_stop_count_{0};
  std::uint64_t safety_stop_count_{0};
  std::uint64_t rejected_command_count_{0};
  std::string last_stop_reason_;
};

}  // namespace humanoid_driver_runtime

#endif  // HUMANOID_DRIVER_RUNTIME__DRIVER_RUNTIME_HPP_
