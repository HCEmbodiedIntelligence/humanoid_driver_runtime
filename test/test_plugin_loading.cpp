// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <gtest/gtest.h>

#include <memory>

#include "humanoid_driver_interface/robot_driver_plugin.hpp"
#include "pluginlib/class_loader.hpp"

namespace hdi = humanoid_driver_interface;

TEST(PluginLoading, LoadsMockByDeclaredClassName)
{
  pluginlib::ClassLoader<hdi::RobotDriverPlugin> loader(
    "humanoid_driver_interface", "humanoid_driver_interface::RobotDriverPlugin");
  auto plugin = loader.createSharedInstance("humanoid_driver_runtime/MockRobotDriver");
  ASSERT_NE(plugin, nullptr);

  hdi::DriverConfiguration configuration;
  configuration.joints.push_back({"joint", "VendorJoint", "arm"});
  EXPECT_TRUE(plugin->configure(configuration));
  EXPECT_TRUE(plugin->connect());
  EXPECT_TRUE(plugin->activate());
  EXPECT_TRUE(plugin->startJointStream());
  EXPECT_TRUE(plugin->stopAll());
  EXPECT_TRUE(plugin->stopJointStream());
  EXPECT_TRUE(plugin->deactivate());
  EXPECT_TRUE(plugin->disconnect());
}
