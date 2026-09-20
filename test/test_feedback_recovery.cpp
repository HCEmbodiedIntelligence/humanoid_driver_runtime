// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <atomic>
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "humanoid_driver_runtime/driver_runtime.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace hdi = humanoid_driver_interface;
namespace hdr = humanoid_driver_runtime;
using namespace std::chrono_literals;

namespace
{
std::atomic<unsigned int> test_id{0};

class FeedbackRecoveryTest : public ::testing::TestWithParam<std::string>
{
protected:
  static void SetUpTestSuite()
  {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }

  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    const auto name = "runtime_feedback_test_" + std::to_string(test_id++);
    node_ = std::make_shared<rclcpp::Node>(name);
    executor_.add_node(node_);
    state_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      "/" + name + "/state", rclcpp::SensorDataQoS());
    command_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/" + name + "/command", rclcpp::QoS(10).reliable(),
      [this](sensor_msgs::msg::JointState::SharedPtr message) {commands_.push_back(*message);});
    config_.plugin_class = "humanoid_driver_runtime/RosTopicRobotDriver";
    config_.ros_node = node_.get();
    config_.plugin_configuration.joints = {{"joint_a", "vendor_a", "arm"}};
    config_.plugin_configuration.parameters = {
      {"state_topic", "/" + name + "/state"},
      {"command_topic", "/" + name + "/command"},
      {"state_timeout_s", "0.1"}, {"startup_grace_s", "1.0"}};
  }

  void TearDown() override
  {
    runtime_.reset();
    executor_.remove_node(node_);
  }

  bool spinUntil(const std::function<bool()> & predicate)
  {
    const auto end = std::chrono::steady_clock::now() + 2s;
    while (std::chrono::steady_clock::now() < end) {
      executor_.spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return false;
  }

  void publish(const double position)
  {
    sensor_msgs::msg::JointState state;
    state.header.stamp = node_->now();
    state.name = {"vendor_a"};
    state.position = {position};
    state_publisher_->publish(state);
  }

  bool feed(const double position)
  {
    return spinUntil([this, position]() {
      publish(position);
      executor_.spin_some();
      const auto result = runtime_->read();
      return result.successful && result.state.positions == std::vector<double>{position};
    });
  }

  static hdi::JointCommand command()
  {
    hdi::JointCommand command;
    command.joint_names = {"joint_a"};
    command.positions = {0.8};
    return command;
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_subscription_;
  std::vector<sensor_msgs::msg::JointState> commands_;
  hdr::DriverRuntimeConfig config_;
  std::unique_ptr<hdr::DriverRuntime> runtime_;
};

TEST_P(FeedbackRecoveryTest, TimeoutHoldsOnceThenFreshFeedbackRestoresReadAndNewCommands)
{
  runtime_ = std::make_unique<hdr::DriverRuntime>(config_);
  ASSERT_TRUE(feed(0.2));
  std::string error;
  ASSERT_TRUE(runtime_->write(command(), error)) << error;
  ASSERT_TRUE(spinUntil([this]() {return commands_.size() == 1;}));
  std::this_thread::sleep_for(150ms);

  if (GetParam() == "read") {
    EXPECT_FALSE(runtime_->read().successful);
  } else if (GetParam() == "health") {
    EXPECT_TRUE(runtime_->status().feedback_waiting);
  } else {
    EXPECT_FALSE(runtime_->write(command(), error));
  }
  const auto waiting = runtime_->status();
  EXPECT_TRUE(waiting.feedback_waiting);
  EXPECT_FALSE(waiting.driver_fault_latched);
  EXPECT_EQ(waiting.feedback_interruption_count, 1U);
  EXPECT_EQ(waiting.feedback_recovery_count, 0U);
  EXPECT_EQ(waiting.safety_stop_count, 1U);
  for (int i = 0; i < 20; ++i) {
    EXPECT_FALSE(runtime_->read().successful);
    EXPECT_FALSE(runtime_->write(command(), error));
    runtime_->enforceWatchdog(std::chrono::steady_clock::now());
    EXPECT_EQ(runtime_->status().safety_stop_count, 1U);
  }
  ASSERT_TRUE(spinUntil([this]() {return commands_.size() == 2;}));
  EXPECT_EQ(commands_.back().position, std::vector<double>{0.2});

  ASSERT_TRUE(feed(0.35));
  const auto recovered = runtime_->status();
  EXPECT_FALSE(recovered.feedback_waiting);
  EXPECT_FALSE(recovered.driver_fault_latched);
  EXPECT_TRUE(recovered.watchdog_stopped);
  EXPECT_EQ(recovered.feedback_recovery_count, 1U);
  ASSERT_TRUE(spinUntil([this]() {return commands_.size() == 3;}));
  EXPECT_EQ(commands_.back().position, std::vector<double>{0.35});
  EXPECT_NE(commands_.back().position, command().positions);  // No replay of the old target.

  ASSERT_TRUE(runtime_->write(command(), error)) << error;
  ASSERT_TRUE(spinUntil([this]() {return commands_.size() == 4;}));
  EXPECT_EQ(commands_.back().position, command().positions);
  EXPECT_FALSE(runtime_->status().watchdog_stopped);

  std::this_thread::sleep_for(150ms);
  EXPECT_FALSE(runtime_->read().successful);
  ASSERT_TRUE(feed(0.4));
  EXPECT_EQ(runtime_->status().feedback_interruption_count, 2U);
  EXPECT_EQ(runtime_->status().feedback_recovery_count, 2U);
}

INSTANTIATE_TEST_SUITE_P(
  TimeoutEntryPoints, FeedbackRecoveryTest, ::testing::Values("read", "health", "write"));

TEST_F(FeedbackRecoveryTest, FirstCompleteFeedbackCanArriveAfterStartupGrace)
{
  config_.plugin_configuration.parameters["startup_grace_s"] = "0.01";
  runtime_ = std::make_unique<hdr::DriverRuntime>(config_);
  std::this_thread::sleep_for(30ms);
  EXPECT_TRUE(runtime_->status().feedback_waiting);
  EXPECT_FALSE(runtime_->read().successful);
  ASSERT_TRUE(feed(0.3));
  EXPECT_FALSE(runtime_->status().driver_fault_latched);
  EXPECT_FALSE(runtime_->status().feedback_waiting);
  EXPECT_EQ(runtime_->status().feedback_recovery_count, 1U);
}

TEST_F(FeedbackRecoveryTest, MalformedFeedbackCannotEndWait)
{
  runtime_ = std::make_unique<hdr::DriverRuntime>(config_);
  ASSERT_TRUE(feed(0.2));
  std::this_thread::sleep_for(150ms);
  EXPECT_FALSE(runtime_->read().successful);
  publish(std::numeric_limits<double>::quiet_NaN());
  executor_.spin_some();
  EXPECT_FALSE(runtime_->read().successful);
  EXPECT_TRUE(runtime_->status().feedback_waiting);
  EXPECT_EQ(runtime_->status().feedback_recovery_count, 0U);
  ASSERT_TRUE(feed(0.25));
  EXPECT_EQ(runtime_->status().feedback_recovery_count, 1U);
}

TEST_F(FeedbackRecoveryTest, ExplicitFaultDuringWaitCannotBeClearedByFreshFeedback)
{
  runtime_ = std::make_unique<hdr::DriverRuntime>(config_);
  ASSERT_TRUE(feed(0.2));
  std::this_thread::sleep_for(150ms);
  EXPECT_FALSE(runtime_->read().successful);
  runtime_->stop("hardware fault", true);
  ASSERT_TRUE(spinUntil([this]() {
    publish(0.3);
    executor_.spin_some();
    return runtime_->status().health.communication_ok;
  }));
  EXPECT_FALSE(runtime_->read().successful);
  EXPECT_TRUE(runtime_->status().driver_fault_latched);
  EXPECT_EQ(runtime_->status().last_stop_reason, "hardware fault");
  EXPECT_EQ(runtime_->status().feedback_recovery_count, 0U);
  std::string error;
  EXPECT_FALSE(runtime_->write(command(), error));
}
}  // namespace
