// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "humanoid_driver_runtime/ros_topic_robot_driver.hpp"

namespace hdi = humanoid_driver_interface;
namespace hmsd = humanoid_driver_runtime;
using namespace std::chrono_literals;

namespace
{

class RosTopicRobotDriverTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("ros_topic_robot_driver_test");
    executor_.add_node(node_);
    state_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      "/test_vendor/joint_state", rclcpp::SensorDataQoS());
    command_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/test_vendor/joint_command", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        received_commands_.push_back(*message);
      });
  }

  void TearDown() override
  {
    executor_.remove_node(node_);
    command_subscription_.reset();
    state_publisher_.reset();
    node_.reset();
  }

  hdi::DriverConfiguration configuration() const
  {
    hdi::DriverConfiguration result;
    result.joints = {
      {"logical_left", "vendor_left", "left_arm", -2.0, 0.1},
      {"logical_right", "vendor_right", "right_arm", 0.5, -0.2},
    };
    result.parameters = {
      {"state_topic", "/test_vendor/joint_state"},
      {"command_topic", "/test_vendor/joint_command"},
      {"state_timeout_s", "0.25"},
      {"startup_grace_s", "1.0"},
    };
    return result;
  }

  void spinUntil(const std::function<bool()> & predicate)
  {
    const auto deadline = std::chrono::steady_clock::now() + 500ms;
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      std::this_thread::sleep_for(1ms);
    }
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_subscription_;
  std::vector<sensor_msgs::msg::JointState> received_commands_;
};

TEST_F(RosTopicRobotDriverTest, MapsNamesAndConvertsVendorCoordinatesBothWays)
{
  hmsd::RosTopicRobotDriver driver;
  ASSERT_TRUE(driver.attachRosNode(*node_));
  ASSERT_TRUE(driver.configure(configuration()));
  ASSERT_TRUE(driver.connect());
  ASSERT_TRUE(driver.activate());
  ASSERT_TRUE(driver.startJointStream());

  hdi::JointState ignored;
  const auto waiting = driver.readJointState(ignored);
  EXPECT_FALSE(waiting);
  EXPECT_EQ(waiting.error, hdi::DriverError::kNoFeedback);

  sensor_msgs::msg::JointState vendor_state;
  vendor_state.name = {"vendor_right", "unused_joint", "vendor_left"};
  vendor_state.position = {0.4, 9.0, 0.2};
  vendor_state.velocity = {0.6, 9.0, -0.3};
  vendor_state.effort = {2.0, 9.0, 6.0};
  state_publisher_->publish(vendor_state);
  spinUntil([&driver, &ignored]() {return static_cast<bool>(driver.readJointState(ignored));});

  ASSERT_TRUE(driver.readJointState(ignored));
  EXPECT_EQ(ignored.joint_names, (std::vector<std::string>{"logical_left", "logical_right"}));
  EXPECT_NEAR(ignored.positions[0], -0.3, 1e-12);
  EXPECT_NEAR(ignored.positions[1], 0.0, 1e-12);
  EXPECT_NEAR(ignored.velocities[0], 0.6, 1e-12);
  EXPECT_NEAR(ignored.velocities[1], 0.3, 1e-12);
  EXPECT_NEAR(ignored.efforts[0], -3.0, 1e-12);
  EXPECT_NEAR(ignored.efforts[1], 4.0, 1e-12);

  hdi::JointCommand command;
  command.joint_names = {"logical_right", "logical_left"};
  command.positions = {0.3, -0.5};
  command.velocities = {0.2, -0.4};
  command.efforts = {1.5, 2.0};
  ASSERT_TRUE(driver.writeJointCommand(command));
  spinUntil([this]() {return !received_commands_.empty();});

  ASSERT_EQ(received_commands_.size(), 1U);
  const auto & vendor_command = received_commands_.front();
  EXPECT_EQ(vendor_command.name, (std::vector<std::string>{"vendor_right", "vendor_left"}));
  ASSERT_EQ(vendor_command.position.size(), 2U);
  EXPECT_NEAR(vendor_command.position[0], 1.0, 1e-12);
  EXPECT_NEAR(vendor_command.position[1], 0.3, 1e-12);
  ASSERT_EQ(vendor_command.velocity.size(), 2U);
  EXPECT_NEAR(vendor_command.velocity[0], 0.4, 1e-12);
  EXPECT_NEAR(vendor_command.velocity[1], 0.2, 1e-12);
  ASSERT_EQ(vendor_command.effort.size(), 2U);
  EXPECT_NEAR(vendor_command.effort[0], 0.75, 1e-12);
  EXPECT_NEAR(vendor_command.effort[1], -4.0, 1e-12);

  EXPECT_TRUE(driver.stopAll());
  EXPECT_TRUE(driver.stopJointStream());
  EXPECT_TRUE(driver.deactivate());
  EXPECT_TRUE(driver.disconnect());
}

}  // namespace
