#ifndef PATROLBOT_SCAN_TIMING_HPP
#define PATROLBOT_SCAN_TIMING_HPP

#include <vector>

namespace patrolbot {

// The largest age permitted for any reading in a scan.  ArLMS2xx may assign
// deinterlaceTime independently to alternating beams, so the newest reading
// is not sufficient evidence that the complete scan is fresh.
const long kMaxScanAgeMs = 500;

// Return the age of the oldest reading when the complete scan is fresh, or -1
// when there are no readings, an age is invalid, or any reading is too old.
// Keeping this helper independent of ARIA makes the freshness contract easy to
// test without opening a device or connecting to a robot.
inline long oldestScanAgeMs(const std::vector<long>& ages_ms) {
  if (ages_ms.empty()) return -1;

  long oldest = 0;
  for (std::vector<long>::const_iterator it = ages_ms.begin();
       it != ages_ms.end(); ++it) {
    if (*it < 0 || *it > kMaxScanAgeMs) return -1;
    if (*it > oldest) oldest = *it;
  }
  return oldest;
}

}  // namespace patrolbot

#endif  // PATROLBOT_SCAN_TIMING_HPP
