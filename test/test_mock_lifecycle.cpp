// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include "humanoid_driver_interface/types.hpp"
#include "humanoid_driver_runtime/mock_robot_driver.hpp"

namespace hdi = humanoid_driver_interface;

namespace
{

hdi::DriverConfiguration configuration()
{
  hdi::DriverConfiguration result;
  result.joints = {
    {"joint_a", "VendorA", "arm"},
    {"joint_b", "VendorB", "arm"},
  };
  return result;
}

TEST(MockLifecycle, EnforcesOrderedLifecycleAndReportsHealth)
{
  humanoid_driver_runtime::MockRobotDriver driver;
  EXPECT_FALSE(driver.connect());
  ASSERT_TRUE(driver.configure(configuration()));
  ASSERT_TRUE(driver.connect());
  EXPECT_TRUE(driver.health().connected);
  ASSERT_TRUE(driver.activate());
  ASSERT_TRUE(driver.startJointStream());

  hdi::JointState state;
  ASSERT_TRUE(driver.readJointState(state));
  EXPECT_EQ(state.joint_names, (std::vector<std::string>{"joint_a", "joint_b"}));

  ASSERT_TRUE(driver.stopAll());
  EXPECT_EQ(driver.stopCount(), 1U);
  ASSERT_TRUE(driver.stopJointStream());
  ASSERT_TRUE(driver.deactivate());
  ASSERT_TRUE(driver.disconnect());
  EXPECT_FALSE(driver.health().connected);
}

TEST(MockLifecycle, RejectsInvalidConfiguration)
{
  humanoid_driver_runtime::MockRobotDriver driver;
  auto duplicate = configuration();
  duplicate.joints[1].logical_name = duplicate.joints[0].logical_name;
  EXPECT_FALSE(driver.configure(duplicate));

  auto bad_parameter = configuration();
  bad_parameter.parameters["velocity_limit_rad_s"] = "0";
  EXPECT_FALSE(driver.configure(bad_parameter));
}

}  // namespace
