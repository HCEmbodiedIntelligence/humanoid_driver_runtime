// Copyright 2026 czy
// SPDX-License-Identifier: LicenseRef-Proprietary

#ifndef HUMANOID_DRIVER_RUNTIME__MOCK_ROBOT_DRIVER_HPP_
#define HUMANOID_DRIVER_RUNTIME__MOCK_ROBOT_DRIVER_HPP_

#include <chrono>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "humanoid_driver_interface/robot_driver_plugin.hpp"

namespace humanoid_driver_runtime
{

class MockRobotDriver final : public humanoid_driver_interface::RobotDriverPlugin
{
public:
  MockRobotDriver() = default;
  ~MockRobotDriver() override = default;

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

  // Runtime fault controls are intentionally public for deterministic integration tests.
  void setDisconnectInjected(bool enabled);
  void setWriteFailureInjected(bool enabled);
  std::uint64_t stopCount() const;

private:
  using TimePoint = std::chrono::steady_clock::time_point;

  struct PendingTarget
  {
    TimePoint apply_at;
    std::vector<double> positions;
  };

  void advanceLocked(TimePoint now);
  void integrateLocked(TimePoint end);
  bool communicationAvailableLocked() const;

  mutable std::mutex mutex_;
  humanoid_driver_interface::DriverConfiguration configuration_;
  std::unordered_map<std::string, std::size_t> joint_indices_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> target_positions_;
  std::deque<PendingTarget> pending_targets_;
  TimePoint last_update_{std::chrono::steady_clock::now()};

  double velocity_limit_rad_s_{std::numeric_limits<double>::infinity()};
  double latency_seconds_{0.0};
  double position_error_rad_{0.0};
  double initial_position_rad_{0.0};

  bool configured_{false};
  bool connected_{false};
  bool active_{false};
  bool streaming_{false};
  bool stopped_{true};
  bool inject_disconnect_{false};
  bool inject_write_failure_{false};
  std::uint64_t stop_count_{0};
  std::uint64_t write_count_{0};
};

}  // namespace humanoid_driver_runtime

#endif  // HUMANOID_DRIVER_RUNTIME__MOCK_ROBOT_DRIVER_HPP_
