// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

#include "gtest/gtest.h"
#include "humanoid_driver_runtime/driver_runtime.hpp"

namespace hdi = humanoid_driver_interface;
namespace hmsd = humanoid_driver_runtime;
using namespace std::chrono_literals;

namespace
{

hmsd::DriverRuntimeConfig config(const std::chrono::milliseconds watchdog = 100ms)
{
  hmsd::DriverRuntimeConfig result;
  result.plugin_class = "humanoid_driver_runtime/MockRobotDriver";
  result.command_watchdog = watchdog;
  result.plugin_configuration.joints = {
    {"joint_a", "JointA", "arm"}, {"joint_b", "JointB", "arm"}};
  result.plugin_configuration.parameters = {
    {"velocity_limit_rad_s", "100.0"}, {"initial_position_rad", "0.0"}};
  return result;
}

hdi::JointCommand command()
{
  hdi::JointCommand result;
  result.joint_names = {"joint_a", "joint_b"};
  result.positions = {0.2, -0.3};
  result.velocities = {0.0, 0.0};
  result.accelerations = {0.0, 0.0};
  return result;
}

TEST(DriverRuntime, ReadsMeasuredFeedbackAndWritesValidatedCommand)
{
  hmsd::DriverRuntime runtime(config());
  const auto initial = runtime.read();
  ASSERT_TRUE(initial.successful) << initial.message;
  EXPECT_EQ(initial.state.joint_names, (std::vector<std::string>{"joint_a", "joint_b"}));

  std::string error;
  EXPECT_TRUE(runtime.write(command(), error)) << error;
  std::this_thread::sleep_for(10ms);
  const auto measured = runtime.read();
  ASSERT_TRUE(measured.successful) << measured.message;
  EXPECT_NEAR(measured.state.positions[0], 0.2, 1e-3);
  EXPECT_NEAR(measured.state.positions[1], -0.3, 1e-3);
}

TEST(DriverRuntime, RejectsUnknownDuplicateNonfiniteAndMismatchedCommands)
{
  hmsd::DriverRuntime runtime(config());
  std::string error;

  auto input = command();
  input.joint_names[1] = "unknown";
  EXPECT_FALSE(runtime.write(input, error));
  EXPECT_NE(error.find("unknown"), std::string::npos);

  input = command();
  input.joint_names[1] = "joint_a";
  EXPECT_FALSE(runtime.write(input, error));
  EXPECT_NE(error.find("duplicate"), std::string::npos);

  input = command();
  input.positions[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(runtime.write(input, error));
  EXPECT_NE(error.find("NaN"), std::string::npos);

  input = command();
  input.positions.pop_back();
  EXPECT_FALSE(runtime.write(input, error));
  EXPECT_NE(error.find("length"), std::string::npos);
  EXPECT_EQ(runtime.status().rejected_command_count, 4U);
}

TEST(DriverRuntime, WatchdogStopsAndFreshFinalRtcCommandRecovers)
{
  hmsd::DriverRuntime runtime(config(20ms));
  std::this_thread::sleep_for(30ms);
  runtime.enforceWatchdog(std::chrono::steady_clock::now());
  auto status = runtime.status();
  EXPECT_TRUE(status.watchdog_stopped);
  EXPECT_EQ(status.watchdog_stop_count, 1U);
  EXPECT_NE(status.last_stop_reason.find("watchdog"), std::string::npos);

  std::string error;
  EXPECT_TRUE(runtime.write(command(), error)) << error;
  EXPECT_FALSE(runtime.status().watchdog_stopped);
}

TEST(DriverRuntime, RequestedPluginFailureIsExplicitAndHasNoFallback)
{
  auto invalid = config();
  invalid.plugin_class = "humanoid_driver_runtime/DoesNotExist";
  EXPECT_THROW(hmsd::DriverRuntime runtime(invalid), std::runtime_error);
}

TEST(DriverRuntime, LoadsPluginFromExplicitDescriptionPath)
{
  auto explicit_config = config();
  explicit_config.plugin_xml_paths = {HUMANOID_RUNTIME_TEST_PLUGIN_XML};
  hmsd::DriverRuntime runtime(explicit_config);
  EXPECT_TRUE(runtime.read().successful);
}

TEST(DriverRuntime, RejectsInvalidExplicitPluginDescriptionPaths)
{
  auto invalid = config();
  invalid.plugin_xml_paths = {"relative/plugins.xml"};
  EXPECT_THROW(hmsd::DriverRuntime runtime(invalid), std::invalid_argument);

  invalid = config();
  invalid.plugin_xml_paths = {"/does/not/exist/plugins.xml"};
  EXPECT_THROW(hmsd::DriverRuntime runtime(invalid), std::invalid_argument);
}

TEST(DriverRuntime, ConfigurationRejectsDuplicateMappings)
{
  auto invalid = config();
  invalid.plugin_configuration.joints[1].logical_name = "joint_a";
  EXPECT_THROW(hmsd::DriverRuntime runtime(invalid), std::invalid_argument);
}

}  // namespace
