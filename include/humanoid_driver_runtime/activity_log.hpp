// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#ifndef HUMANOID_DRIVER_RUNTIME__ACTIVITY_LOG_HPP_
#define HUMANOID_DRIVER_RUNTIME__ACTIVITY_LOG_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace humanoid_driver_runtime
{

// Observational only: never used to accept commands, enforce watchdogs or stop hardware.
class ActivityTransition
{
public:
  using Clock = std::chrono::steady_clock;

  void observe(const std::string & state)
  {
    if (state != state_) {
      state_ = state;
      ++changes_;
      if (path_.size() < 8U) {
        path_.push_back(state);
      } else {
        path_.back() = state;
      }
    }
  }

  std::string poll(const std::string & source, const Clock::time_point now)
  {
    if (changes_ == 0 || (have_log_ && now - last_log_ < std::chrono::seconds(1))) {
      return {};
    }
    auto message = source + ": " + logged_;
    for (std::size_t i = 0; i < path_.size(); ++i) {
      if (i + 1 == path_.size() && changes_ > path_.size()) {
        message += " -> [" + std::to_string(changes_ - path_.size()) + " changes omitted]";
      }
      message += " -> " + path_[i];
    }
    message += "; changes=" + std::to_string(changes_);
    logged_ = state_;
    changes_ = 0;
    path_.clear();
    last_log_ = now;
    have_log_ = true;
    return message;
  }

private:
  std::string state_;
  std::string logged_{"unknown"};
  std::uint64_t changes_{0};
  std::vector<std::string> path_;
  bool have_log_{false};
  Clock::time_point last_log_;
};

class JointActivityLog
{
public:
  using Clock = ActivityTransition::Clock;

  JointActivityLog(
    const std::vector<std::string> & names, const std::vector<std::string> & groups,
    const std::chrono::duration<double> command_timeout)
  : command_timeout_(command_timeout)
  {
    for (std::size_t i = 0; i < names.size(); ++i) {
      groups_[groups.at(i)].names.push_back(names[i]);
    }
    for (auto & item : groups_) {
      item.second.anchor.resize(item.second.names.size());
      item.second.positions.resize(item.second.names.size());
    }
  }

  void sample(
    const std::vector<std::string> & names, const std::vector<double> & positions,
    const Clock::time_point now)
  {
    for (auto & item : groups_) {
      auto & group = item.second;
      bool complete = true;
      double delta = 0.;
      for (std::size_t i = 0; i < group.names.size(); ++i) {
        const auto found = std::find(names.begin(), names.end(), group.names[i]);
        const auto index = static_cast<std::size_t>(found - names.begin());
        if (found == names.end() || index >= positions.size() || !std::isfinite(positions[index])) {
          complete = false;
          break;
        }
        group.positions[i] = positions[index];
        delta = std::max(delta, std::abs(group.positions[i] - group.anchor[i]));
      }
      if (!complete) {
        group.have_sample = false;
        group.moving = false;
        group.measured.observe("unavailable");
        continue;
      }
      if (!group.have_sample) {
        group.anchor = group.positions;
        group.moving = false;
      } else if (delta >= 0.005) {
        // Compare against an anchor, so slow accumulated motion is not lost at 100 Hz.
        group.anchor = group.positions;
        group.last_change = now;
        group.moving = true;
      }
      group.have_sample = true;
      if (group.moving && now - group.last_change >= std::chrono::milliseconds(500)) {
        group.moving = false;
      }
      group.measured.observe(group.moving ? "moving" : "stationary");
    }
  }

  void unavailable()
  {
    for (auto & item : groups_) {
      item.second.have_sample = false;
      item.second.moving = false;
      item.second.measured.observe("unavailable");
    }
  }

  void acceptedCommand(const std::vector<std::string> & names, const Clock::time_point now)
  {
    for (auto & item : groups_) {
      auto & group = item.second;
      if (std::any_of(names.begin(), names.end(), [&group](const auto & name) {
          return std::find(group.names.begin(), group.names.end(), name) != group.names.end();
        }))
      {
        group.have_command = true;
        group.last_command = now;
        group.commands.observe("receiving");
      }
    }
  }

  std::vector<std::string> poll(const Clock::time_point now)
  {
    std::vector<std::string> result;
    for (auto & item : groups_) {
      auto & group = item.second;
      if (!group.have_command || now - group.last_command > command_timeout_) {
        group.commands.observe("stopped");
      }
      if (!group.have_sample) {
        group.measured.observe("unavailable");
      }
      for (const auto & message : {
          group.commands.poll("joint_command." + item.first, now),
          group.measured.poll("joint_feedback." + item.first, now)})
      {
        if (!message.empty()) {
          result.push_back(message);
        }
      }
    }
    return result;
  }

private:
  struct Group
  {
    std::vector<std::string> names;
    std::vector<double> anchor;
    std::vector<double> positions;
    bool have_sample{false};
    bool moving{false};
    bool have_command{false};
    Clock::time_point last_change;
    Clock::time_point last_command;
    ActivityTransition measured;
    ActivityTransition commands;
  };
  std::map<std::string, Group> groups_;
  std::chrono::duration<double> command_timeout_;
};

}  // namespace humanoid_driver_runtime

#endif  // HUMANOID_DRIVER_RUNTIME__ACTIVITY_LOG_HPP_
