#ifndef PATROLBOT_MANUAL_TURN_SCAN_HPP
#define PATROLBOT_MANUAL_TURN_SCAN_HPP

#include <cmath>
#include <vector>

namespace patrolbot {

struct LocalScanPoint {
  double x_m;
  double y_m;
  double range_m;
};

// Same existing laser rotation stop zone/min_points/range floor as
// nav2_params.yaml and bridge SCAN_RANGE_MIN. Coordinates are ARIA's local
// robot coordinates, so the physical sensor mounting is applied by ARIA,
// not by an assumed ROS map or a second hard-coded transform.
inline bool manualTurnScanValid(const std::vector<LocalScanPoint>& points, long age_ms) {
  if (age_ms < 0 || age_ms > 500 || points.size() < 2) return false;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const LocalScanPoint& p = points[i];
    if (!std::isfinite(p.x_m) || !std::isfinite(p.y_m) ||
        !std::isfinite(p.range_m) || p.range_m < 0) return false;
  }
  return true;
}

inline bool manualTurnScanClear(const std::vector<LocalScanPoint>& points,
                                long age_ms) {
  if (!manualTurnScanValid(points, age_ms)) return false;
  unsigned int hits = 0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const LocalScanPoint& p = points[i];
    if (p.range_m >= 0.10 && std::fabs(p.x_m) <= 0.30 &&
        std::fabs(p.y_m) <= 0.30 && ++hits >= 8) return false;
  }
  return true;
}

}  // namespace patrolbot
#endif
