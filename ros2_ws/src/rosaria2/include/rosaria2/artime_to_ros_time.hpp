#ifndef ROSARIA2_INCLUDE_ROSARIA2_ARTIMETOROSTIME_HPP_
#define ROSARIA2_INCLUDE_ROSARIA2_ARTIMETOROSTIME_HPP_

#include <rclcpp/rclcpp.hpp>
#ifdef ADEPT_PKG
#include "ariaUtil.h"
#else
#include "Aria/ariaUtil.h"
#endif

rclcpp::Time toROSTime(const ArTime& t, const rclcpp::Node& node) {
  // ARIA/ARNL times are in reference to an arbitrary starting time, not OS clock, so find the time elapsed between now and t
  // to adjust the time stamp in ROS time vs. now accordingly.
  ArTime arianow;
  const double dtsec = (double) t.mSecSince(arianow) / 1000.0;
  //printf("was %f seconds ago\n", dtsec);
  return rclcpp::Time(node.now().seconds() - dtsec);
}

#endif  // ROSARIA2_INCLUDE_ROSARIA2_ARTIMETOROSTIME_HPP_