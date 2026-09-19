#include <cassert>
#include <cstdlib>
#include <iostream>
#include <memory>
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rcl/time.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

int main(int argc, char ** argv) {
  assert(std::getenv("PATROLBOT_ISOLATED_TEST") != nullptr);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_util::LifecycleNode>("observation_acceptance");
  auto clock = node->get_clock();
  assert(rcl_enable_ros_time_override(clock->get_clock_handle()) == RCL_RET_OK);
  assert(rcl_set_ros_time_override(clock->get_clock_handle(), 1000000000LL) == RCL_RET_OK);
  tf2_ros::Buffer tf(clock);
  tf2_ros::TransformListener listener(tf);
  {
    nav2_costmap_2d::ObservationBuffer buffer(
      node, "scan", 0.0, 0.5, 0.0, 2.0, 20.0, 0.0, 20.0, 0.0,
      tf, "map", "map", tf2::durationFromSec(0.1));
    assert(buffer.isCurrent());
    constexpr long long checks = 8LL * 60 * 60 * 5;
    for (long long i = 0; i < checks; ++i) {
      assert(rcl_set_ros_time_override(clock->get_clock_handle(),
        2000000000LL + i * 200000000LL) == RCL_RET_OK);
      assert(!buffer.isCurrent());
    }
    // Real bufferCloud recovery path; no direct timestamp reset shortcut.
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = "map";
    cloud.header.stamp = node->now();
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(1);
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    *x = 1.0; *y = 0.0; *z = 0.1;
    buffer.bufferCloud(cloud);
    assert(buffer.isCurrent());
    assert(buffer.isCurrent());
    std::cout << "ACCEPTED simulated_outage_seconds=28800 stale_checks=" << checks
              << " false_current_failures=0 recovery_without_restart=true\n";
  }
  node.reset();
  rclcpp::shutdown();
}
