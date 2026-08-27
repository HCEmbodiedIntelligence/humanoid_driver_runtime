// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "humanoid_driver_runtime/diagnostics_runtime.hpp"

#include <utility>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace humanoid_driver_runtime
{

diagnostic_msgs::msg::DiagnosticArray DiagnosticsRuntime::makeDriverReport(
  const DriverRuntimeStatus & runtime_status, const rclcpp::Time & stamp,
  const std::string & node_name, const std::string & plugin_class) const
{
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = node_name + ": driver";
  status.hardware_id = plugin_class;
  status.level = static_cast<std::uint8_t>(runtime_status.health.level);
  status.message = runtime_status.health.message;
  const auto add = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue item;
      item.key = key;
      item.value = value;
      status.values.push_back(std::move(item));
    };
  add("communication_ok", runtime_status.health.communication_ok ? "true" : "false");
  add("connected", runtime_status.health.connected ? "true" : "false");
  add("active", runtime_status.health.active ? "true" : "false");
  add("watchdog_stopped", runtime_status.watchdog_stopped ? "true" : "false");
  add("driver_fault_latched", runtime_status.driver_fault_latched ? "true" : "false");
  add("watchdog_stops", std::to_string(runtime_status.watchdog_stop_count));
  add("safety_stops", std::to_string(runtime_status.safety_stop_count));
  add("rejected_commands", std::to_string(runtime_status.rejected_command_count));
  add("last_stop_reason", runtime_status.last_stop_reason);
  for (const auto & [key, value] : runtime_status.health.details) {
    add(key, value);
  }
  array.status.push_back(std::move(status));
  return array;
}

}  // namespace humanoid_driver_runtime
