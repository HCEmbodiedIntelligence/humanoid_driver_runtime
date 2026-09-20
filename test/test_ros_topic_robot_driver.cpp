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
  vendor_state.header.stamp = node_->now();
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


TEST_F(RosTopicRobotDriverTest, SourceTimeSurvivesCachedReadsAndReplaysCannotRenewIt)
{
  hmsd::RosTopicRobotDriver driver;
  ASSERT_TRUE(driver.attachRosNode(*node_));
  ASSERT_TRUE(driver.configure(configuration()));
  ASSERT_TRUE(driver.connect());
  ASSERT_TRUE(driver.activate());
  ASSERT_TRUE(driver.startJointStream());
  spinUntil([this]() {return state_publisher_->get_subscription_count() > 0;});
  sensor_msgs::msg::JointState message;
  message.name = {"vendor_left", "vendor_right"};
  message.position = {0.2, 0.4};
  message.header.stamp = node_->now() - rclcpp::Duration::from_seconds(1.0);
  state_publisher_->publish(message);
  spinUntil([&]() {return driver.health().details.at("state_topic") == "/test_vendor/joint_state";});
  executor_.spin_some();
  hdi::JointState state;
  EXPECT_FALSE(driver.readJointState(state));
  message.header.stamp = node_->now() - rclcpp::Duration::from_seconds(.05);
  const auto before = std::chrono::steady_clock::now();
  state_publisher_->publish(message);
  spinUntil([&]() {return static_cast<bool>(driver.readJointState(state));});
  ASSERT_TRUE(driver.readJointState(state));
  EXPECT_LT(state.sample_time, before - 40ms);
  const auto sample = state.sample_time;
  message.position = {9., 9.};
  for (int n = 0; n < 30; ++n) {
    state_publisher_->publish(message);  // Duplicate source stamp with altered data.
    executor_.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_FALSE(driver.readJointState(state));
  EXPECT_EQ(state.sample_time, sample);
  message.header.stamp = node_->now();
  message.position = {0.3, 0.4};
  state_publisher_->publish(message);
  spinUntil([&]() {return static_cast<bool>(driver.readJointState(state));});
  EXPECT_TRUE(driver.readJointState(state));
  EXPECT_GT(state.sample_time, sample);
}

TEST_F(RosTopicRobotDriverTest, GroupPositionTopicsPreserveOrderScalingAndOtherJointTargets)
{
  auto config = configuration();
  config.joints = {{"a", "va", "arm_x", -2., .1}, {"b", "vb", "arm_x", 1., 0.},
    {"c", "vc", "arm_y", 1., 0.}};
  config.parameters.erase("command_topic");
  config.parameters["command_topic.arm_x"] = "/test_vendor/group_x";
  config.parameters["command_topic.arm_y"] = "/test_vendor/group_y";
  std::vector<std::vector<double>> x, y;
  auto xs = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/test_vendor/group_x", 10, [&](const std_msgs::msg::Float64MultiArray & m) {x.push_back(m.data);});
  auto ys = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/test_vendor/group_y", 10, [&](const std_msgs::msg::Float64MultiArray & m) {y.push_back(m.data);});
  hmsd::RosTopicRobotDriver driver;
  ASSERT_TRUE(driver.attachRosNode(*node_));
  ASSERT_TRUE(driver.configure(config));
  ASSERT_TRUE(driver.connect());
  ASSERT_TRUE(driver.activate());
  ASSERT_TRUE(driver.startJointStream());
  spinUntil([this]() {return state_publisher_->get_subscription_count() > 0;});
  sensor_msgs::msg::JointState message;
  message.header.stamp = node_->now();
  message.name = {"vc", "vb", "va"};
  message.position = {.8, .4, .2};
  state_publisher_->publish(message);
  hdi::JointState state;
  spinUntil([&]() {return static_cast<bool>(driver.readJointState(state));});
  hdi::JointCommand cmd;
  cmd.joint_names = {"a"}; cmd.positions = {.3};
  ASSERT_TRUE(driver.writeJointCommand(cmd));
  spinUntil([&]() {return x.size() == 1;});
  ASSERT_EQ(x.size(), 1U);
  EXPECT_NEAR(x.back()[0], -.1, 1e-12);
  EXPECT_DOUBLE_EQ(x.back()[1], .4);
  EXPECT_TRUE(y.empty());
  cmd.joint_names = {"b"}; cmd.positions = {.7};
  ASSERT_TRUE(driver.writeJointCommand(cmd));
  spinUntil([&]() {return x.size() == 2;});
  EXPECT_NEAR(x.back()[0], -.1, 1e-12);
  EXPECT_DOUBLE_EQ(x.back()[1], .7);
  ASSERT_TRUE(driver.stopAll());
  spinUntil([&]() {return !y.empty() && x.size() == 3;});
  EXPECT_DOUBLE_EQ(x.back()[0], .2);
  EXPECT_DOUBLE_EQ(x.back()[1], .4);
  EXPECT_EQ(y.back(), std::vector<double>{.8});
}

}  // namespace
