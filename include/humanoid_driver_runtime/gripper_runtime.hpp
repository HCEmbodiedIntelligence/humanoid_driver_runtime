// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#ifndef HUMANOID_DRIVER_RUNTIME__GRIPPER_RUNTIME_HPP_
#define HUMANOID_DRIVER_RUNTIME__GRIPPER_RUNTIME_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "humanoid_driver_interface/gripper_driver_plugin.hpp"
#include "humanoid_driver_runtime/driver_runtime.hpp"
#include "pluginlib/class_loader.hpp"

namespace rclcpp
{
class Node;
}  // namespace rclcpp

namespace humanoid_driver_runtime
{

struct GripperRuntimeConfig
{
  std::string plugin_class;
  std::vector<std::string> plugin_xml_paths;
  humanoid_driver_interface::GripperConfiguration plugin_configuration;
  std::chrono::milliseconds command_watchdog{250};
  rclcpp::Node * ros_node{nullptr};
};

struct GripperReadResult
{
  bool successful{false};
  humanoid_driver_interface::GripperState state;
  std::string message;
};

// Owns exactly one gripper plugin. A plugin may expose multiple grippers, which keeps a bimanual
// pair atomic without making grippers part of the robot-motion joint contract.
class GripperRuntime
{
public:
  explicit GripperRuntime(const GripperRuntimeConfig & config);
  ~GripperRuntime();

  GripperRuntime(const GripperRuntime &) = delete;
  GripperRuntime & operator=(const GripperRuntime &) = delete;

  GripperReadResult read();
  bool write(const humanoid_driver_interface::GripperCommand & command, std::string & error);
  void enforceWatchdog(std::chrono::steady_clock::time_point now);
  void stop(const std::string & reason, bool driver_fault = false);
  DriverRuntimeStatus status();

private:
  using Plugin = humanoid_driver_interface::GripperDriverPlugin;
  using Command = humanoid_driver_interface::GripperCommand;
  using State = humanoid_driver_interface::GripperState;
  using Mapping = humanoid_driver_interface::GripperMapping;

  static void validateConfiguration(const GripperRuntimeConfig & config);
  bool validateCommand(const Command & command, std::string & reason) const;
  bool validateState(const State & state, std::string & reason) const;
  static bool allFinite(const std::vector<double> & values);
  void stopLocked(const std::string & reason, bool driver_fault);
  void shutdownLocked() noexcept;

  std::unique_ptr<pluginlib::ClassLoader<Plugin>> plugin_loader_;
  std::shared_ptr<Plugin> plugin_;
  std::vector<Mapping> mappings_;
  std::unordered_map<std::string, Mapping> mapping_by_logical_name_;
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

#endif  // HUMANOID_DRIVER_RUNTIME__GRIPPER_RUNTIME_HPP_
