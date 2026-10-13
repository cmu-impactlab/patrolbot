#include "../lidar_power.hpp"

#include <cassert>
#include <cstdio>
#include <fstream>
#include <limits>
#include <unistd.h>

int main() {
  using namespace patrolbot;
  LidarPower uncommissioned(false);
  assert(!uncommissioned.supported());
  assert(!uncommissioned.ready());
  assert(!uncommissioned.requestAllowed(0, 0, true, false));
  uncommissioned.observe(20, 100);
  assert(uncommissioned.ready());
  uncommissioned.observe(-1, 200);
  assert(!uncommissioned.ready());

  LidarPower lidar(true);
  assert(lidar.requestAllowed(0, 0, true, false));
  assert(!lidar.requestAllowed(.006, 0, true, false));
  assert(!lidar.requestAllowed(0, .011, true, false));
  assert(!lidar.requestAllowed(0, 0, false, false));
  assert(!lidar.requestAllowed(0, 0, true, true));
  assert(!lidar.requestAllowed(std::numeric_limits<double>::quiet_NaN(), 0, true, false));
  lidar.onRequested(1000);
  assert(lidar.state() == LIDAR_STARTING);
  assert(!lidar.requestAllowed(0, 0, true, false));
  lidar.observe(100, 1050);  // Cached acquisition predates explicit ON.
  assert(!lidar.ready());
  lidar.observe(20, 1100);
  assert(lidar.ready());
  lidar.observe(501, 1200);
  assert(lidar.state() == LIDAR_FAULT);
  lidar.observe(20, 1300);  // A transient scan loss can recover readiness.
  assert(lidar.ready());
  lidar.offRequested();
  lidar.observe(0, 1400);
  assert(lidar.state() == LIDAR_OFF);
  assert(!lidar.shouldReconnect());
  lidar.onRequested(2000);
  assert(lidar.shouldReconnect());
  lidar.observe(-1, 62001);
  assert(lidar.state() == LIDAR_FAULT);
  lidar.commandFailed();
  lidar.observe(0, 62002);
  assert(!lidar.ready());
  assert(!lidar.shouldReconnect());
  lidar.onRequested(63000);
  lidar.observe(0, 63001);
  assert(lidar.ready());

  char directory[] = "/tmp/patrolbot-lidar-test-XXXXXX";
  assert(mkdtemp(directory));
  const std::string path = std::string(directory) + "/state";
  const std::string boot = "123e4567-e89b-12d3-a456-426614174000";
  assert(loadLidarState(path, boot) == LIDAR_UNKNOWN);
  assert(saveLidarState(path, boot, LIDAR_OFF));
  assert(loadLidarState(path, boot) == LIDAR_OFF);
  assert(loadLidarState(path, "aaaaaaaa-bbbb-4ccc-8ddd-eeeeeeeeeeee") == LIDAR_UNKNOWN);
  LidarPower restarted(true, loadLidarState(path, boot));
  restarted.observe(0, 1);
  assert(restarted.state() == LIDAR_OFF);
  assert(saveLidarState(path, boot, LIDAR_FAULT));
  LidarPower ambiguous(true, loadLidarState(path, boot));
  ambiguous.observe(0, 1);
  assert(!ambiguous.ready());
  assert(saveLidarState(path, boot, LIDAR_STARTING));
  assert(loadLidarState(path, boot) == LIDAR_UNKNOWN);
  { std::ofstream corrupt(path.c_str()); corrupt << "broken\n"; }
  assert(loadLidarState(path, boot) == LIDAR_FAULT);
  assert(loadLidarState(path, "") == LIDAR_FAULT);
  assert(!saveLidarState(path + "/missing", boot, LIDAR_OFF));
  assert(!saveLidarState(path, "", LIDAR_OFF));
  std::remove(path.c_str());
  rmdir(directory);
  std::fprintf(stderr, "lidar_power_test: state, commissioning, stationary, ageing and restart checks passed\n");
}
