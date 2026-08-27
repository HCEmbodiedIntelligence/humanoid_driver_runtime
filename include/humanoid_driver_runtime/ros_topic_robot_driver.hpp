// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#ifndef HUMANOID_DRIVER_RUNTIME__ROS_TOPIC_ROBOT_DRIVER_HPP_
#define HUMANOID_DRIVER_RUNTIME__ROS_TOPIC_ROBOT_DRIVER_HPP_

#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "humanoid_driver_interface/ros2_driver_plugin.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace humanoid_driver_runtime
{

// Adapts the common vendor ROS 2 convention of publishing measured JointState and accepting a
// position setpoint JointState. All vendor-specific names, joint order, axis signs, zero offsets,
// and topic names are supplied through DriverConfiguration rather than compiled into the plugin.
class RosTopicRobotDriver final : public humanoid_driver_interface::Ros2DriverPlugin
{
public:
  RosTopicRobotDriver() = default;
  ~RosTopicRobotDriver() override = default;

  humanoid_driver_interface::DriverResult attachRosNode(rclcpp::Node & node) override;
  humanoid_driver_interface::DriverResult configure(
    const humanoid_driver_interface::DriverConfiguration & configuration) override;
  humanoid_driver_interface::DriverResult connect() override;
  humanoid_driver_interface::DriverResult disconnect() override;
  humanoid_driver_interface::DriverResult activate() override;
  humanoid_driver_interface::DriverResult deactivate() override;
  humanoid_driver_interface::DriverResult readJointState(
    humanoid_driver_interface::JointState & state) override;
  humanoid_driver_interface::DriverResult startJointStream() override;
  humanoid_driver_interface::DriverResult writeJointCommand(
    const humanoid_driver_interface::JointCommand & command) override;
  humanoid_driver_interface::DriverResult stopJointStream() override;
  humanoid_driver_interface::DriverResult stopAll() override;
  humanoid_driver_interface::DriverHealth health() override;

private:
  using Clock = std::chrono::steady_clock;
  using Mapping = humanoid_driver_interface::JointMapping;
  using Result = humanoid_driver_interface::DriverResult;

  void stateCallback(const sensor_msgs::msg::JointState::SharedPtr message);
  bool stateFreshLocked(Clock::time_point now) const;
  Result publishCommandLocked(const humanoid_driver_interface::JointCommand & command);
  Result publishHoldLocked();

  mutable std::mutex mutex_;
  rclcpp::Node * node_{nullptr};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_publisher_;

  std::vector<Mapping> mappings_;
  std::unordered_map<std::string, std::size_t> mapping_by_logical_name_;
  std::unordered_map<std::string, std::size_t> mapping_by_vendor_name_;
  humanoid_driver_interface::JointState latest_state_;
  Clock::time_point configured_at_{Clock::now()};
  Clock::time_point last_state_received_{Clock::now()};
  std::chrono::milliseconds state_timeout_{250};
  std::chrono::milliseconds startup_grace_{3000};
  std::string state_topic_;
  std::string command_topic_;
  std::string last_feedback_error_;

  bool configured_{false};
  bool connected_{false};
  bool active_{false};
  bool streaming_{false};
  bool stopped_{true};
  bool have_state_{false};
};

}  // namespace humanoid_driver_runtime

#endif  // HUMANOID_DRIVER_RUNTIME__ROS_TOPIC_ROBOT_DRIVER_HPP_
