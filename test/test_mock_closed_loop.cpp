// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <thread>

#include "humanoid_driver_interface/types.hpp"
#include "humanoid_driver_runtime/mock_robot_driver.hpp"

namespace hdi = humanoid_driver_interface;
using namespace std::chrono_literals;

namespace
{

hdi::DriverConfiguration makeConfiguration()
{
  hdi::DriverConfiguration result;
  result.joints.push_back({"joint", "VendorJoint", "arm"});
  return result;
}

hdi::JointCommand command(double position)
{
  hdi::JointCommand result;
  result.joint_names = {"joint"};
  result.vendor_joint_names = {"VendorJoint"};
  result.vendor_groups = {"arm"};
  result.positions = {position};
  return result;
}

TEST(MockClosedLoop, SupportsIdealTrackingLatencyAndFeedbackError)
{
  auto configuration = makeConfiguration();
  configuration.parameters["latency_s"] = "0.03";
  configuration.parameters["position_error_rad"] = "0.01";
  humanoid_driver_runtime::MockRobotDriver driver;
  ASSERT_TRUE(driver.configure(configuration));
  ASSERT_TRUE(driver.connect());
  ASSERT_TRUE(driver.activate());
  ASSERT_TRUE(driver.startJointStream());
  ASSERT_TRUE(driver.writeJointCommand(command(0.75)));

  hdi::JointState state;
  ASSERT_TRUE(driver.readJointState(state));
  EXPECT_NEAR(state.positions.at(0), 0.01, 1e-6);
  std::this_thread::sleep_for(40ms);
  ASSERT_TRUE(driver.readJointState(state));
  EXPECT_NEAR(state.positions.at(0), 0.76, 1e-6);
}

TEST(MockClosedLoop, RateLimitsReachesFeedbackAndHoldsAfterStop)
{
  auto configuration = makeConfiguration();
  configuration.parameters["velocity_limit_rad_s"] = "4.0";
  humanoid_driver_runtime::MockRobotDriver driver;
  ASSERT_TRUE(driver.configure(configuration));
  ASSERT_TRUE(driver.connect());
  ASSERT_TRUE(driver.activate());
  ASSERT_TRUE(driver.startJointStream());
  ASSERT_TRUE(driver.writeJointCommand(command(0.4)));

  hdi::JointState state;
  const auto deadline = std::chrono::steady_clock::now() + 300ms;
  do {
    std::this_thread::sleep_for(10ms);
    ASSERT_TRUE(driver.readJointState(state));
  } while (std::abs(state.positions.at(0) - 0.4) > 1e-3 &&
    std::chrono::steady_clock::now() < deadline);
  EXPECT_NEAR(state.positions.at(0), 0.4, 1e-3);

  ASSERT_TRUE(driver.writeJointCommand(command(-0.4)));
  std::this_thread::sleep_for(25ms);
  ASSERT_TRUE(driver.stopAll());
  ASSERT_TRUE(driver.readJointState(state));
  const double held_position = state.positions.at(0);
  EXPECT_GT(held_position, -0.4);
  std::this_thread::sleep_for(60ms);
  ASSERT_TRUE(driver.readJointState(state));
  EXPECT_NEAR(state.positions.at(0), held_position, 1e-6);

  driver.setDisconnectInjected(true);
  EXPECT_FALSE(driver.readJointState(state));
  driver.setDisconnectInjected(false);
  driver.setWriteFailureInjected(true);
  EXPECT_FALSE(driver.writeJointCommand(command(0.0)));
}

}  // namespace
