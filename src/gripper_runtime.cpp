// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#include "humanoid_driver_runtime/gripper_runtime.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include "humanoid_driver_interface/ros2_gripper_driver_plugin.hpp"

namespace hdi = humanoid_driver_interface;

namespace humanoid_driver_runtime
{

GripperRuntime::GripperRuntime(const GripperRuntimeConfig & config)
: mappings_(config.plugin_configuration.grippers),
  watchdog_timeout_(config.command_watchdog),
  last_command_time_(std::chrono::steady_clock::now())
{
  validateConfiguration(config);
  plugin_loader_ = std::make_unique<pluginlib::ClassLoader<Plugin>>(
    "humanoid_driver_interface", "humanoid_driver_interface::GripperDriverPlugin", "plugin",
    config.plugin_xml_paths);
  for (const auto & mapping : mappings_) {
    mapping_by_logical_name_.emplace(mapping.logical_name, mapping);
  }
  try {
    plugin_ = plugin_loader_->createSharedInstance(config.plugin_class);
  } catch (const pluginlib::PluginlibException & error) {
    throw std::runtime_error(
            "failed to load requested gripper plugin '" + config.plugin_class + "': " +
            error.what());
  }
  if (!plugin_) {
    throw std::runtime_error("pluginlib returned a null gripper plugin");
  }

  const auto invoke = [this](const char * operation, const auto & callback) {
      const auto result = callback();
      if (!result) {
        shutdownLocked();
        throw std::runtime_error(
                std::string("gripper driver ") + operation + " failed: " + result.message);
      }
    };
  try {
    if (auto * ros_plugin = dynamic_cast<hdi::Ros2GripperDriverPlugin *>(plugin_.get())) {
      if (config.ros_node == nullptr) {
        throw std::invalid_argument("the selected ROS 2 gripper plugin requires the runtime node");
      }
      invoke("ROS node attachment", [&]() {return ros_plugin->attachRosNode(*config.ros_node);});
    }
    invoke("configure", [&]() {return plugin_->configure(config.plugin_configuration);});
    invoke("connect", [&]() {return plugin_->connect();});
    invoke("activate", [&]() {return plugin_->activate();});
    invoke("stream start", [&]() {return plugin_->startGripperStream();});
    active_ = true;
  } catch (const std::exception &) {
    shutdownLocked();
    throw;
  }
}

GripperRuntime::~GripperRuntime()
{
  std::lock_guard<std::mutex> lock(mutex_);
  shutdownLocked();
}

GripperReadResult GripperRuntime::read()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || !plugin_) {
    return {false, {}, "gripper runtime is inactive"};
  }
  if (driver_fault_latched_) {
    return {false, {}, "gripper driver fault is latched: " + last_stop_reason_};
  }
  State state;
  try {
    const auto result = plugin_->readGripperState(state);
    if (!result) {
      if (result.error == hdi::DriverError::kNoFeedback) {
        return {false, {}, result.message};
      }
      stopLocked("gripper-state read failed: " + result.message, true);
      return {false, {}, last_stop_reason_};
    }
  } catch (const std::exception & error) {
    stopLocked("gripper-state read exception: " + std::string(error.what()), true);
    return {false, {}, last_stop_reason_};
  }
  std::string reason;
  if (!validateState(state, reason)) {
    stopLocked(reason, true);
    return {false, {}, reason};
  }
  return {true, std::move(state), {}};
}

bool GripperRuntime::write(const Command & command, std::string & error)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || !plugin_) {
    error = "gripper runtime is inactive";
    ++rejected_command_count_;
    return false;
  }
  if (driver_fault_latched_) {
    error = "gripper driver fault is latched: " + last_stop_reason_;
    ++rejected_command_count_;
    return false;
  }
  if (!validateCommand(command, error)) {
    ++rejected_command_count_;
    return false;
  }

  Command adapted = command;
  adapted.vendor_gripper_names.clear();
  adapted.vendor_gripper_names.reserve(command.gripper_names.size());
  for (const auto & name : command.gripper_names) {
    adapted.vendor_gripper_names.push_back(mapping_by_logical_name_.at(name).vendor_name);
  }
  try {
    const auto result = plugin_->writeGripperCommand(adapted);
    if (!result) {
      error = "gripper-command write failed: " + result.message;
      stopLocked(error, true);
      return false;
    }
  } catch (const std::exception & exception) {
    error = "gripper-command write exception: " + std::string(exception.what());
    stopLocked(error, true);
    return false;
  }
  last_command_time_ = std::chrono::steady_clock::now();
  watchdog_latched_ = false;
  return true;
}

void GripperRuntime::enforceWatchdog(const std::chrono::steady_clock::time_point now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_ || driver_fault_latched_ || watchdog_latched_) {
    return;
  }
  if (now - last_command_time_ > watchdog_timeout_) {
    watchdog_latched_ = true;
    ++watchdog_stop_count_;
    stopLocked("gripper command watchdog expired", false);
  }
}

void GripperRuntime::stop(const std::string & reason, const bool driver_fault)
{
  std::lock_guard<std::mutex> lock(mutex_);
  stopLocked(reason, driver_fault);
}

DriverRuntimeStatus GripperRuntime::status()
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
    result.health.message = "gripper plugin is not loaded";
    return result;
  }
  try {
    result.health = plugin_->health();
    if (result.health.level == hdi::HealthLevel::kError ||
      result.health.level == hdi::HealthLevel::kStale || !result.health.communication_ok)
    {
      stopLocked("gripper health reports an error: " + result.health.message, true);
      result.driver_fault_latched = true;
      result.safety_stop_count = safety_stop_count_;
      result.last_stop_reason = last_stop_reason_;
    }
  } catch (const std::exception & error) {
    result.health.level = hdi::HealthLevel::kError;
    result.health.message = "gripper health exception: " + std::string(error.what());
    stopLocked(result.health.message, true);
    result.driver_fault_latched = true;
    result.safety_stop_count = safety_stop_count_;
    result.last_stop_reason = last_stop_reason_;
  }
  return result;
}

void GripperRuntime::validateConfiguration(const GripperRuntimeConfig & config)
{
  if (config.plugin_class.empty() || config.command_watchdog.count() <= 0 ||
    config.plugin_configuration.grippers.empty())
  {
    throw std::invalid_argument("gripper plugin, positive watchdog, and mappings are required");
  }
  std::unordered_set<std::string> xml_paths;
  for (const auto & path_text : config.plugin_xml_paths) {
    const std::filesystem::path path(path_text);
    if (!path.is_absolute() || !std::filesystem::is_regular_file(path) ||
      !xml_paths.insert(std::filesystem::weakly_canonical(path).string()).second)
    {
      throw std::invalid_argument(
              "explicit gripper plugin XML paths must be unique absolute regular files");
    }
  }
  std::unordered_set<std::string> logical_names;
  std::unordered_set<std::string> vendor_names;
  for (const auto & mapping : config.plugin_configuration.grippers) {
    if (mapping.logical_name.empty() || mapping.vendor_name.empty() ||
      (mapping.position_unit != "m" && mapping.position_unit != "rad") ||
      !std::isfinite(mapping.vendor_to_logical_scale) ||
      std::abs(mapping.vendor_to_logical_scale) < 1.0e-12 ||
      !std::isfinite(mapping.vendor_to_logical_offset) ||
      !logical_names.insert(mapping.logical_name).second ||
      !vendor_names.insert(mapping.vendor_name).second)
    {
      throw std::invalid_argument("gripper mappings contain an invalid or duplicate entry");
    }
  }
  for (const auto & [key, value] : config.plugin_configuration.parameters) {
    if (key.empty() || value.empty()) {
      throw std::invalid_argument("gripper plugin parameters require non-empty keys and values");
    }
  }
}

bool GripperRuntime::validateCommand(const Command & command, std::string & reason) const
{
  const auto count = command.gripper_names.size();
  if (count == 0U || command.positions.size() != count ||
    (!command.max_efforts.empty() && command.max_efforts.size() != count) ||
    !allFinite(command.positions) || !allFinite(command.max_efforts) ||
    std::any_of(command.max_efforts.begin(), command.max_efforts.end(),
      [](const double effort) {return effort < 0.0;}))
  {
    reason = "gripper command fields are empty, mismatched, non-finite, or negative";
    return false;
  }
  std::unordered_set<std::string> names;
  for (const auto & name : command.gripper_names) {
    if (!names.insert(name).second || mapping_by_logical_name_.count(name) == 0U) {
      reason = "gripper command contains an unknown or duplicate name '" + name + "'";
      return false;
    }
  }
  return true;
}

bool GripperRuntime::validateState(const State & state, std::string & reason) const
{
  const auto count = state.gripper_names.size();
  if (count != mappings_.size() || state.positions.size() != count ||
    (!state.efforts.empty() && state.efforts.size() != count) ||
    !allFinite(state.positions) || !allFinite(state.efforts))
  {
    reason = "gripper driver returned inconsistent or non-finite state fields";
    return false;
  }
  std::unordered_set<std::string> names;
  for (const auto & name : state.gripper_names) {
    if (!names.insert(name).second || mapping_by_logical_name_.count(name) == 0U) {
      reason = "gripper driver returned an unknown or duplicate name '" + name + "'";
      return false;
    }
  }
  return true;
}

bool GripperRuntime::allFinite(const std::vector<double> & values)
{
  return std::all_of(
    values.begin(), values.end(), [](const double value) {return std::isfinite(value);});
}

void GripperRuntime::stopLocked(const std::string & reason, const bool driver_fault)
{
  if (driver_fault) {
    driver_fault_latched_ = true;
  }
  ++safety_stop_count_;
  last_stop_reason_ = reason;
  if (plugin_) {
    try {
      (void)plugin_->stopAll();
    } catch (const std::exception &) {
    }
  }
}

void GripperRuntime::shutdownLocked() noexcept
{
  active_ = false;
  if (!plugin_) {
    return;
  }
  try {
    (void)plugin_->stopAll();
    (void)plugin_->stopGripperStream();
    (void)plugin_->deactivate();
    (void)plugin_->disconnect();
  } catch (const std::exception &) {
  }
  plugin_.reset();
}

}  // namespace humanoid_driver_runtime
