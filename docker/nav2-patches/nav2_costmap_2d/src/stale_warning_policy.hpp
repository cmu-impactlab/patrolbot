#ifndef PATROLBOT_NAV2_STALE_WARNING_POLICY_HPP
#define PATROLBOT_NAV2_STALE_WARNING_POLICY_HPP

#include <chrono>
#include <cstddef>

namespace patrolbot_nav2_patch {

enum class StaleWarningEvent {
  None,
  Stale,
  Recovered
};

struct StaleWarningResult {
  StaleWarningEvent event;
  std::size_t stale_checks;
};

class StaleWarningPolicy {
public:
  typedef std::chrono::steady_clock Clock;
  typedef Clock::time_point TimePoint;

  explicit StaleWarningPolicy(
    std::chrono::seconds repeat_interval = std::chrono::seconds(30))
  : repeat_interval_(repeat_interval), stale_(false), have_time_(false),
    last_time_(), last_warning_(), stale_checks_(0)
  {
  }

  StaleWarningResult observe(bool current, TimePoint now) {
    if (have_time_ && now < last_time_) {
      now = last_time_;
    }
    last_time_ = now;
    have_time_ = true;

    if (current) {
      if (!stale_) {
        return {StaleWarningEvent::None, stale_checks_};
      }
      stale_ = false;
      return {StaleWarningEvent::Recovered, stale_checks_};
    }

    ++stale_checks_;
    if (!stale_) {
      stale_ = true;
      last_warning_ = now;
      return {StaleWarningEvent::Stale, stale_checks_};
    }
    if (now - last_warning_ >= repeat_interval_) {
      last_warning_ = now;
      return {StaleWarningEvent::Stale, stale_checks_};
    }
    return {StaleWarningEvent::None, stale_checks_};
  }

private:
  const std::chrono::seconds repeat_interval_;
  bool stale_;
  bool have_time_;
  TimePoint last_time_;
  TimePoint last_warning_;
  std::size_t stale_checks_;
};

}  // namespace patrolbot_nav2_patch

#endif  // PATROLBOT_NAV2_STALE_WARNING_POLICY_HPP
