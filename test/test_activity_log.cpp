// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#include <algorithm>
#include <chrono>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "humanoid_driver_runtime/activity_log.hpp"

using namespace std::chrono_literals;
using humanoid_driver_runtime::ActivityTransition;
using humanoid_driver_runtime::JointActivityLog;

namespace
{
const auto start = JointActivityLog::Clock::time_point{};

bool contains(const std::vector<std::string> & messages, const std::string & text)
{
  return std::any_of(messages.begin(), messages.end(), [&text](const auto & message) {
    return message.find(text) != std::string::npos;
  });
}

TEST(ActivityLog, CommandAcceptanceDoesNotImplyMeasuredMovement)
{
  JointActivityLog log({"left_a", "right_a"}, {"left", "right"}, 100ms);
  log.sample({"right_a", "left_a"}, {1., 0.}, start);
  auto messages = log.poll(start);
  EXPECT_TRUE(contains(messages, "joint_feedback.left: unknown -> stationary"));
  EXPECT_TRUE(contains(messages, "joint_feedback.right: unknown -> stationary"));
  log.acceptedCommand({"right_a"}, start + 1s);
  messages = log.poll(start + 1s);
  ASSERT_EQ(messages.size(), 1U);
  EXPECT_TRUE(contains(messages, "joint_command.right: stopped -> receiving"));
  log.sample({"left_a", "right_a"}, {0., 1.01}, start + 1100ms);
  messages = log.poll(start + 1100ms);
  EXPECT_TRUE(contains(messages, "joint_feedback.right: stationary -> moving"));
  EXPECT_FALSE(contains(messages, "joint_feedback.left"));
  log.sample({"left_a", "right_a"}, {0., 1.01}, start + 2100ms);
  messages = log.poll(start + 2100ms);
  EXPECT_TRUE(contains(messages, "joint_feedback.right: moving -> stationary"));
  EXPECT_TRUE(contains(messages, "joint_command.right: receiving -> stopped"));
  EXPECT_TRUE(log.poll(start + 1h).empty());
}

TEST(ActivityLog, IgnoresNoiseButAccumulatesSlowMotionAndResetsAfterFeedbackLoss)
{
  JointActivityLog log({"a"}, {"arm"}, 100ms);
  log.sample({"a"}, {0.}, start);
  log.poll(start);
  for (int i = 1; i <= 100; ++i) {
    log.sample({"a"}, {i % 2 ? .0001 : -.0001}, start + i*10ms);
    EXPECT_TRUE(log.poll(start + i*10ms).empty());
  }
  for (int i = 1; i < 6; ++i) {
    log.sample({"a"}, {i*.001}, start + 1s + i*10ms);
  }
  EXPECT_TRUE(contains(log.poll(start + 1100ms), "stationary -> moving"));
  log.unavailable();
  EXPECT_TRUE(contains(log.poll(start + 3s), "moving -> unavailable"));
  // Reconnection at a different position must not be reported as observed movement.
  log.sample({"a"}, {1.}, start + 4s);
  EXPECT_TRUE(contains(log.poll(start + 4s), "unavailable -> stationary"));
  log.sample({}, {}, start + 5s);
  EXPECT_TRUE(contains(log.poll(start + 5s), "stationary -> unavailable"));
}

TEST(ActivityLog, RepeatedStatesStaySilentAndFlappingIsCoalesced)
{
  ActivityTransition log;
  log.observe("stationary");
  EXPECT_FALSE(log.poll("arm", start).empty());
  for (int i = 1; i < 100; ++i) {
    log.observe(i % 2 ? "moving" : "stationary");
    EXPECT_TRUE(log.poll("arm", start + i*10ms).empty());
  }
  EXPECT_NE(log.poll("arm", start + 1s).find("changes=99"), std::string::npos);
  log.observe("moving");
  EXPECT_TRUE(log.poll("arm", start + 10s).empty());
}

TEST(ActivityLog, RetainsBriefMovementEvenWhenStateReturnsToStationary)
{
  ActivityTransition log;
  log.observe("stationary");
  log.poll("arm", start);
  log.observe("moving");
  EXPECT_TRUE(log.poll("arm", start + 100ms).empty());
  log.observe("stationary");
  EXPECT_NE(
    log.poll("arm", start + 1s).find("stationary -> moving -> stationary"), std::string::npos);
}
}  // namespace
