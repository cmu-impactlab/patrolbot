#ifndef PATROLBOT_DOCKING_PLUGINS__PATROLBOT_CHARGING_DOCK_HPP_
#define PATROLBOT_DOCKING_PLUGINS__PATROLBOT_CHARGING_DOCK_HPP_

#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "opennav_docking_core/charging_dock.hpp"
#include "patrolbot_interfaces/msg/base_state.hpp"
#include "patrolbot_interfaces/srv/charge_release.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace patrolbot_docking_plugins
{

class PatrolBotChargingDock : public opennav_docking_core::ChargingDock
{
public:
  PatrolBotChargingDock() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf) override;
  void cleanup() override;
  void activate() override {}
  void deactivate() override;

  geometry_msgs::msg::PoseStamped getStagingPose(
    const geometry_msgs::msg::Pose & pose, const std::string & frame) override;
  bool getRefinedPose(
    geometry_msgs::msg::PoseStamped & pose, std::string id) override;
  bool isDocked() override;
  bool isCharging() override;
  bool disableCharging() override;
  bool hasStoppedCharging() override;

private:
  using ChargeRelease = patrolbot_interfaces::srv::ChargeRelease;

  void baseStateCallback(
    const patrolbot_interfaces::msg::BaseState::SharedPtr message);
  void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr message);
  bool telemetryFreshLocked(const rclcpp::Time & now) const;
  static std::string makeRequestId();

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Subscription<patrolbot_interfaces::msg::BaseState>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Client<ChargeRelease>::SharedPtr release_client_;

  mutable std::mutex mutex_;
  patrolbot_interfaces::msg::BaseState state_;
  bool state_received_{false};
  bool battery_received_{false};
  rclcpp::Time state_received_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time battery_received_at_{0, 0, RCL_ROS_TIME};
  bool release_pending_{false};
  std::string release_request_id_;
  uint64_t release_session_{0};
  rclcpp::Time release_started_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Client<ChargeRelease>::SharedFuture release_future_;

  double telemetry_timeout_{0.5};
  double release_timeout_{7.0};
  double staging_x_offset_{-0.7};
  double staging_yaw_offset_{0.0};
};

}  // namespace patrolbot_docking_plugins

#endif  // PATROLBOT_DOCKING_PLUGINS__PATROLBOT_CHARGING_DOCK_HPP_
