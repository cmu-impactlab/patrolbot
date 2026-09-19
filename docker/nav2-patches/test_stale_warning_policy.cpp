// Standalone tests for the ABI-safe stale warning policy.

#include "stale_warning_policy.hpp"

#include <cassert>
#include <chrono>
#include <cstddef>
#include <iostream>

namespace {

using Policy = patrolbot_nav2_patch::StaleWarningPolicy;
using Event = patrolbot_nav2_patch::StaleWarningEvent;

Policy::TimePoint at_ms(long long milliseconds) {
  return Policy::TimePoint(std::chrono::milliseconds(milliseconds));
}

void test_first_stale_is_immediate() {
  Policy policy;
  assert(policy.observe(true, at_ms(0)).event == Event::None);
  assert(policy.observe(false, at_ms(200)).event == Event::Stale);
  assert(policy.observe(false, at_ms(400)).event == Event::None);
}

void test_repeat_is_bounded_over_eight_hours_at_five_hz() {
  Policy policy;
  std::size_t stale_events = 0;
  const long long checks = 8LL * 60 * 60 * 5;
  for (long long i = 0; i < checks; ++i) {
    if (policy.observe(false, at_ms(i * 200)).event == Event::Stale) {
      ++stale_events;
    }
  }
  // First warning plus one approximately every 30 seconds. The exact final
  // boundary is intentionally covered by the inclusive policy comparison.
  assert(stale_events == 960);
  const auto boundary = policy.observe(false, at_ms(checks * 200));
  assert(boundary.event == Event::Stale);
  assert(boundary.stale_checks ==
         static_cast<std::size_t>(checks + 1));
}

void test_recovery_is_emitted_once() {
  Policy policy;
  assert(policy.observe(true, at_ms(0)).stale_checks == 0);
  assert(policy.observe(false, at_ms(1)).event == Event::Stale);
  assert(policy.observe(true, at_ms(2)).event == Event::Recovered);
  assert(policy.observe(true, at_ms(3)).stale_checks == 1);
  assert(policy.observe(false, at_ms(4)).event == Event::Stale);
  assert(policy.observe(false, at_ms(5)).stale_checks == 3);
}

void test_clock_regression_does_not_repeat_early() {
  Policy policy;
  assert(policy.observe(false, at_ms(1000)).event == Event::Stale);
  assert(policy.observe(false, at_ms(900)).event == Event::None);
  assert(policy.observe(false, at_ms(30999)).event == Event::None);
  assert(policy.observe(false, at_ms(31000)).event == Event::Stale);
}

void test_new_stale_episode_warns_immediately() {
  Policy policy;
  assert(policy.observe(false, at_ms(0)).event == Event::Stale);
  assert(policy.observe(true, at_ms(1)).event == Event::Recovered);
  assert(policy.observe(false, at_ms(2)).event == Event::Stale);
}

}  // namespace

int main() {
  test_first_stale_is_immediate();
  test_repeat_is_bounded_over_eight_hours_at_five_hz();
  test_recovery_is_emitted_once();
  test_clock_regression_does_not_repeat_early();
  test_new_stale_episode_warns_immediately();
  std::cout << "stale warning policy tests passed\n";
  return 0;
}
