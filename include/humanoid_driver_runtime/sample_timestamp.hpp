#ifndef HUMANOID_DRIVER_RUNTIME__SAMPLE_TIMESTAMP_HPP_
#define HUMANOID_DRIVER_RUNTIME__SAMPLE_TIMESTAMP_HPP_

#include <algorithm>
#include <chrono>
#include <cstdint>

namespace humanoid_driver_runtime
{
// Per source stream. Preserve transport age on the local steady clock and do
// not let repeated source samples refresh their age.
class SampleTimestamp
{
public:
  using Clock = std::chrono::steady_clock;
  bool accept(std::int64_t stamp, std::int64_t ros_now, Clock::time_point received,
    std::chrono::nanoseconds max_age, Clock::time_point & sampled)
  {
    if (ros_now < last_ros_now_) {
      last_stamp_ = 0;
    }
    last_ros_now_ = ros_now;
    if (stamp <= 0 || ros_now < 0 || stamp <= last_stamp_) {
      return false;
    }
    const auto age = std::chrono::nanoseconds(ros_now - stamp);
    if (age >= max_age || age < -std::chrono::milliseconds(50)) {
      return false;
    }
    last_stamp_ = stamp;
    sampled = received - std::max(age, std::chrono::nanoseconds::zero());
    return true;
  }
private:
  std::int64_t last_stamp_{0};
  std::int64_t last_ros_now_{0};
};
}  // namespace humanoid_driver_runtime
#endif
