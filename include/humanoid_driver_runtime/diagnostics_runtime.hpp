// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#ifndef HUMANOID_DRIVER_RUNTIME__DIAGNOSTICS_RUNTIME_HPP_
#define HUMANOID_DRIVER_RUNTIME__DIAGNOSTICS_RUNTIME_HPP_

#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "humanoid_driver_runtime/driver_runtime.hpp"
#include "rclcpp/time.hpp"

namespace humanoid_driver_runtime
{

class DiagnosticsRuntime
{
public:
  diagnostic_msgs::msg::DiagnosticArray makeDriverReport(
    const DriverRuntimeStatus & runtime_status, const rclcpp::Time & stamp,
    const std::string & node_name, const std::string & plugin_class) const;
};

}  // namespace humanoid_driver_runtime

#endif  // HUMANOID_DRIVER_RUNTIME__DIAGNOSTICS_RUNTIME_HPP_
