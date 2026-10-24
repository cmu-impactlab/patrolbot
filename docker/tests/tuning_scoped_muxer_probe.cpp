// Real patched NavigatorMuxer, including the mutex shared with NEW goal admission.
#include <atomic>
#include <cassert>
#include <iostream>
#include <string>
#include <thread>
#include "nav2_core/behavior_tree_navigator.hpp"

int main()
{
  const std::string digest(64, 'a'), plan(64, 'b');
  nav2_core::NavigatorMuxer muxer;
  std::string evidence;
  assert(muxer.tryStartNavigating("goal"));
  assert(!muxer.tuningMaintenance("acquire", "busy-job", digest, plan, evidence));
  muxer.stopNavigating("goal");
  assert(muxer.setTuningHold(true));  // Independent seed/admission hold.
  assert(muxer.tuningMaintenance("acquire", "job", digest, plan, evidence));
  assert(!muxer.tuningMaintenance("release", "foreign", digest, plan, evidence));
  assert(!muxer.tuningMaintenance("release", "job", digest, std::string(64, 'c'), evidence));
  assert(muxer.setTuningHold(false));  // Cannot release the exact-job hold.
  assert(!muxer.tryStartNavigating("goal"));
  muxer.startNavigating("legacy-bypass");
  assert(!muxer.isNavigating());
  assert(muxer.setTuningHold(true));
  assert(muxer.tuningMaintenance("release", "job", digest, plan, evidence));
  assert(!muxer.tryStartNavigating("goal"));  // Releasing job cannot clear seed hold.
  assert(muxer.setTuningHold(false));
  assert(muxer.tryStartNavigating("goal"));
  muxer.stopNavigating("goal");
  for (int i = 0; i < 1000; ++i) {
    std::atomic<int> ready{0};
    std::atomic<bool> go{false};
    bool goal = false, held = false;
    const auto job = "race-" + std::to_string(i);
    std::thread a([&]() {
      ++ready; while (!go.load()) {std::this_thread::yield();}
      goal = muxer.tryStartNavigating("racing-goal");
    });
    std::thread b([&]() {
      ++ready; while (!go.load()) {std::this_thread::yield();}
      held = muxer.tuningMaintenance("acquire", job, digest, plan, evidence);
    });
    while (ready.load() != 2) {std::this_thread::yield();}
    go.store(true);
    a.join(); b.join();
    assert(goal != held);  // Exactly one wins; neither silently cancels the other.
    if (goal) {muxer.stopNavigating("racing-goal");}
    if (held) {assert(muxer.tuningMaintenance("release", job, digest, plan, evidence));}
  }
  std::cout << "PASS: exact-job NavigatorMuxer ownership and 1000 admission races\n";
}
