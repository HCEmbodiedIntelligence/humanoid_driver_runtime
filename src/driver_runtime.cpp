// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "humanoid_driver_runtime/driver_runtime.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include "humanoid_driver_interface/ros2_driver_plugin.hpp"

namespace hdi = humanoid_driver_interface;

namespace humanoid_driver_runtime
{

DriverRuntime::DriverRuntime(const DriverRuntimeConfig & config)
: plugin_loader_("humanoid_driver_interface", "humanoid_driver_interface::RobotDriverPlugin"),
  joint_mappings_(config.plugin_configuration.joints),
  watchdog_timeout_(config.command_watchdog),
  last_command_time_(std::chrono::steady_clock::now())
{
  validateConfiguration(config);
  for (const auto & mapping : joint_mappings_) {
    mapping_by_logical_name_.emplace(mapping.logical_name, mapping);
  }

  try {
    plugin_ = plugin_loader_.createSharedInstance(config.plugin_class);
  } catch (const pluginlib::PluginlibException & error) {
    throw std::runtime_error(
            "failed to load requested driver plugin '" + config.plugin_class + "': " +
            error.what());
  }
  if (!plugin_) {
    throw std::runtime_error("pluginlib returned a null driver plugin");
  }

  const auto invoke = [this](
    const char * operation, const auto & callback) {
      const auto result = callback();
      if (!result) {
        shutdownLocked();
        throw std::runtime_error(
                std::string("driver ") + operation + " failed: " + result.message);
      }
  };
  try {
    if (auto * ros_plugin = dynamic_cast<hdi::Ros2DriverPlugin *>(plugin_.get())) {
      if (config.ros_node == nullptr) {
        throw std::invalid_argument(
                "the selected ROS 2 driver plugin requires the runtime ROS node");
      }
      invoke("ROS node attachment", [&]() {return ros_plugin->attachRosNode(*config.ros_node);});
    }
    invoke("configure", [&]() {return plugin_->configure(config.plugin_configuration);});
    invoke("connect", [&]() {return plugin_->connect();});
    invoke("activate", [&]() {return plugin_->activate();});
    invoke("stream start", [&]() {return plugin_->startJointStream();});
    active_ = true;
  } catch (const std::exception &) {
    shutdownLocked();
    throw;
  }
}

DriverRuntime::~DriverRuntime()
{
  std::lock_guard<std::mutex> lock(mutex_);
  shutdownLocked();
}

DriverReadResult DriverRuntime::read()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || !plugin_) {
    return {false, {}, "driver runtime is inactive"};
  }
  if (driver_fault_latched_) {
    return {false, {}, "driver fault is latched: " + last_stop_reason_};
  }
  JointState state;
  try {
    const auto result = plugin_->readJointState(state);
    if (!result) {
      if (result.error == hdi::DriverError::kNoFeedback) {
        // ROS subscriptions cannot deliver a first sample before their owning node enters its
        // executor. The plugin is responsible for converting this into a communication fault
        // once its finite startup grace period has expired.
        return {false, {}, result.message};
      }
      stopLocked("joint-state read failed: " + result.message, true);
      return {false, {}, last_stop_reason_};
    }
  } catch (const std::exception & error) {
    stopLocked("joint-state read exception: " + std::string(error.what()), true);
    return {false, {}, last_stop_reason_};
  }
  std::string reason;
  if (!validateState(state, reason)) {
    stopLocked(reason, true);
    return {false, {}, reason};
  }
  return {true, std::move(state), {}};
}

bool DriverRuntime::write(const JointCommand & command, std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || !plugin_) {
    error = "driver runtime is inactive";
    ++rejected_command_count_;
    return false;
  }
  if (driver_fault_latched_) {
    error = "driver fault is latched: " + last_stop_reason_;
    ++rejected_command_count_;
    return false;
  }
  if (!validateCommand(command, error)) {
    ++rejected_command_count_;
    return false;
  }

  JointCommand adapted = command;
  adapted.vendor_joint_names.clear();
  adapted.vendor_groups.clear();
  adapted.vendor_joint_names.reserve(command.joint_names.size());
  adapted.vendor_groups.reserve(command.joint_names.size());
  for (const auto & name : command.joint_names) {
    const auto & mapping = mapping_by_logical_name_.at(name);
    adapted.vendor_joint_names.push_back(mapping.vendor_name);
    adapted.vendor_groups.push_back(mapping.vendor_group);
  }

  try {
    const auto result = plugin_->writeJointCommand(adapted);
    if (!result) {
      error = "joint-command write failed: " + result.message;
      stopLocked(error, true);
      return false;
    }
  } catch (const std::exception & exception) {
    error = "joint-command write exception: " + std::string(exception.what());
    stopLocked(error, true);
    return false;
  }
  last_command_time_ = std::chrono::steady_clock::now();
  watchdog_latched_ = false;
  return true;
}

void DriverRuntime::enforceWatchdog(const std::chrono::steady_clock::time_point now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || driver_fault_latched_ || watchdog_latched_) {
    return;
  }
  if (now - last_command_time_ > watchdog_timeout_) {
    watchdog_latched_ = true;
    ++watchdog_stop_count_;
    stopLocked("command watchdog expired", false);
  }
}

void DriverRuntime::stop(const std::string & reason, const bool driver_fault)
{
  std::lock_guard<std::mutex> lock(mutex_);
  stopLocked(reason, driver_fault);
}

DriverRuntimeStatus DriverRuntime::status()
{
  std::lock_guard<std::mutex> lock(mutex_);
  DriverRuntimeStatus result;
  result.watchdog_stopped = watchdog_latched_;
  result.driver_fault_latched = driver_fault_latched_;
  result.watchdog_stop_count = watchdog_stop_count_;
  result.safety_stop_count = safety_stop_count_;
  result.rejected_command_count = rejected_command_count_;
  result.last_stop_reason = last_stop_reason_;
  if (!plugin_) {
    result.health.message = "driver plugin is not loaded";
    return result;
  }
  try {
    result.health = plugin_->health();
    if (result.health.level == hdi::HealthLevel::kError ||
      result.health.level == hdi::HealthLevel::kStale || !result.health.communication_ok)
    {
      stopLocked("driver health reports an error: " + result.health.message, true);
      result.driver_fault_latched = true;
      result.safety_stop_count = safety_stop_count_;
      result.last_stop_reason = last_stop_reason_;
    }
  } catch (const std::exception & error) {
    result.health.level = hdi::HealthLevel::kError;
    result.health.communication_ok = false;
    result.health.message = "health exception: " + std::string(error.what());
    stopLocked(result.health.message, true);
    result.driver_fault_latched = true;
    result.safety_stop_count = safety_stop_count_;
    result.last_stop_reason = last_stop_reason_;
  }
  return result;
}

const std::vector<DriverRuntime::JointMapping> & DriverRuntime::jointMappings() const noexcept
{
  return joint_mappings_;
}

void DriverRuntime::validateConfiguration(const DriverRuntimeConfig & config)
{
  if (config.plugin_class.empty()) {
    throw std::invalid_argument("plugin_class is required; no fallback driver is allowed");
  }
  if (config.command_watchdog.count() <= 0) {
    throw std::invalid_argument("command watchdog must be positive");
  }
  if (config.plugin_configuration.joints.empty()) {
    throw std::invalid_argument("at least one driver joint mapping is required");
  }
  std::unordered_set<std::string> logical_names;
  std::unordered_set<std::string> vendor_keys;
  for (const auto & mapping : config.plugin_configuration.joints) {
    if (mapping.logical_name.empty() || mapping.vendor_name.empty() ||
      mapping.vendor_group.empty())
    {
      throw std::invalid_argument("driver joint mapping entries must be non-empty");
    }
    if (!logical_names.insert(mapping.logical_name).second) {
      throw std::invalid_argument("duplicate logical driver joint '" + mapping.logical_name + "'");
    }
    if (!vendor_keys.insert(mapping.vendor_group + "\n" + mapping.vendor_name).second) {
      throw std::invalid_argument(
              "duplicate vendor driver joint '" + mapping.vendor_group + "/" +
              mapping.vendor_name + "'");
    }
  }
  for (const auto & [key, value] : config.plugin_configuration.parameters) {
    if (key.empty() || value.empty()) {
      throw std::invalid_argument("driver plugin parameters must have non-empty keys and values");
    }
  }
}

bool DriverRuntime::validateCommand(const JointCommand & command, std::string & reason) const
{
  const auto count = command.joint_names.size();
  if (count == 0U || command.positions.size() != count ||
    (!command.velocities.empty() && command.velocities.size() != count) ||
    (!command.accelerations.empty() && command.accelerations.size() != count) ||
    (!command.efforts.empty() && command.efforts.size() != count))
  {
    reason = "driver command field lengths do not match non-empty joint_names";
    return false;
  }
  if (!allFinite(command.positions) || !allFinite(command.velocities) ||
    !allFinite(command.accelerations) || !allFinite(command.efforts))
  {
    reason = "driver command contains NaN or Inf";
    return false;
  }
  std::unordered_set<std::string> names;
  for (const auto & name : command.joint_names) {
    if (!names.insert(name).second) {
      reason = "duplicate driver command joint '" + name + "'";
      return false;
    }
    if (mapping_by_logical_name_.count(name) == 0U) {
      reason = "unknown driver command joint '" + name + "'";
      return false;
    }
  }
  return true;
}

bool DriverRuntime::validateState(const JointState & state, std::string & reason) const
{
  const auto count = state.joint_names.size();
  if (count != joint_mappings_.size() || state.positions.size() != count ||
    (!state.velocities.empty() && state.velocities.size() != count) ||
    (!state.efforts.empty() && state.efforts.size() != count))
  {
    reason = "driver returned inconsistent joint-state lengths";
    return false;
  }
  if (!allFinite(state.positions) || !allFinite(state.velocities) || !allFinite(state.efforts)) {
    reason = "driver returned NaN or Inf";
    return false;
  }
  std::unordered_set<std::string> names;
  for (const auto & name : state.joint_names) {
    if (mapping_by_logical_name_.count(name) == 0U || !names.insert(name).second) {
      reason = "driver returned unknown or duplicate joint '" + name + "'";
      return false;
    }
  }
  return true;
}

bool DriverRuntime::allFinite(const std::vector<double> & values)
{
  return std::all_of(
    values.begin(), values.end(), [](const double value) {return std::isfinite(value);});
}

void DriverRuntime::stopLocked(const std::string & reason, const bool driver_fault)
{
  if (driver_fault) {
    driver_fault_latched_ = true;
  }
  ++safety_stop_count_;
  last_stop_reason_ = reason;
  if (!plugin_) {
    return;
  }
  try {
    (void)plugin_->stopAll();
  } catch (const std::exception &) {
    // The original failure remains authoritative; diagnostics expose it.
  }
}

void DriverRuntime::shutdownLocked() noexcept
{
  active_ = false;
  if (!plugin_) {
    return;
  }
  try {
    (void)plugin_->stopAll();
    (void)plugin_->stopJointStream();
    (void)plugin_->deactivate();
    (void)plugin_->disconnect();
  } catch (const std::exception &) {
  }
  plugin_.reset();
}

}  // namespace humanoid_driver_runtime
