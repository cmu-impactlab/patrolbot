#include "patrolbot_docking_plugins/patrolbot_charging_dock.hpp"

#include <cmath>
#include <iomanip>
#include <random>
#include <sstream>
#include <utility>

#include "patrolbot_docking_plugins/charge_state.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace patrolbot_docking_plugins
{

void PatrolBotChargingDock::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf)
{
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("unable to lock docking server lifecycle node");
  }
  name_ = name;
  tf_buffer_ = std::move(tf);

  const auto parameter = [this](const std::string & suffix, const double value) {
      const auto full_name = name_ + "." + suffix;
      if (!node_->has_parameter(full_name)) {
        node_->declare_parameter(full_name, value);
      }
      return node_->get_parameter(full_name).as_double();
    };
  telemetry_timeout_ = parameter("telemetry_timeout", 0.5);
  release_timeout_ = parameter("release_timeout", 7.0);
  staging_x_offset_ = parameter("staging_x_offset", -0.7);
  staging_yaw_offset_ = parameter("staging_yaw_offset", 0.0);

  state_sub_ = node_->create_subscription<patrolbot_interfaces::msg::BaseState>(
    "/patrolbot/base_state", rclcpp::QoS(10),
    std::bind(&PatrolBotChargingDock::baseStateCallback, this, std::placeholders::_1));
  battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
    "/battery", rclcpp::QoS(10),
    std::bind(&PatrolBotChargingDock::batteryCallback, this, std::placeholders::_1));
  release_client_ = node_->create_client<ChargeRelease>("/patrolbot/charge_release");
}

void PatrolBotChargingDock::cleanup()
{
  std::scoped_lock lock(mutex_);
  release_pending_ = false;
  release_future_ = {};
  release_client_.reset();
  battery_sub_.reset();
  state_sub_.reset();
  tf_buffer_.reset();
  node_.reset();
}

void PatrolBotChargingDock::deactivate()
{
  std::scoped_lock lock(mutex_);
  release_pending_ = false;
  release_future_ = {};
}

geometry_msgs::msg::PoseStamped PatrolBotChargingDock::getStagingPose(
  const geometry_msgs::msg::Pose & pose, const std::string & frame)
{
  tf2::Transform dock;
  tf2::fromMsg(pose, dock);
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, staging_yaw_offset_);
  tf2::Transform offset(rotation, tf2::Vector3(staging_x_offset_, 0.0, 0.0));
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = frame;
  result.header.stamp = node_->now();
  const tf2::Transform staging = dock * offset;
  result.pose.position.x = staging.getOrigin().x();
  result.pose.position.y = staging.getOrigin().y();
  result.pose.position.z = staging.getOrigin().z();
  result.pose.orientation = tf2::toMsg(staging.getRotation());
  return result;
}

bool PatrolBotChargingDock::getRefinedPose(
  geometry_msgs::msg::PoseStamped & pose, std::string /* id */)
{
  pose.header.stamp = node_->now();
  return true;
}

void PatrolBotChargingDock::baseStateCallback(
  const patrolbot_interfaces::msg::BaseState::SharedPtr message)
{
  std::scoped_lock lock(mutex_);
  state_ = *message;
  state_received_ = true;
  state_received_at_ = node_->now();
}

void PatrolBotChargingDock::batteryCallback(
  const sensor_msgs::msg::BatteryState::SharedPtr /* message */)
{
  std::scoped_lock lock(mutex_);
  battery_received_ = true;
  battery_received_at_ = node_->now();
}

bool PatrolBotChargingDock::telemetryFreshLocked(const rclcpp::Time & now) const
{
  return state_received_ && battery_received_ && state_.link_connected &&
         state_.hardware_state_valid &&
         (now - state_received_at_).seconds() <= telemetry_timeout_ &&
         (now - battery_received_at_).seconds() <= telemetry_timeout_;
}

bool PatrolBotChargingDock::isDocked()
{
  std::scoped_lock lock(mutex_);
  return telemetryFreshLocked(node_->now()) &&
         isDockedState(state_.dock_state_valid, state_.dock_state);
}

bool PatrolBotChargingDock::isCharging()
{
  std::scoped_lock lock(mutex_);
  return telemetryFreshLocked(node_->now()) &&
         isChargingState(
           state_.charge_state_valid, state_.charge_state,
           state_.dock_state_valid, state_.dock_state);
}

bool PatrolBotChargingDock::hasStoppedCharging()
{
  std::scoped_lock lock(mutex_);
  return telemetryFreshLocked(node_->now()) &&
         isReleasedState(state_.charge_state_valid, state_.charge_state);
}

bool PatrolBotChargingDock::disableCharging()
{
  std::scoped_lock lock(mutex_);
  const auto now = node_->now();
  if (!telemetryFreshLocked(now)) {
    return false;
  }
  if (isReleasedState(state_.charge_state_valid, state_.charge_state)) {
    release_pending_ = false;
    release_future_ = {};
    return true;
  }
  if (!isChargingState(
      state_.charge_state_valid, state_.charge_state,
      state_.dock_state_valid, state_.dock_state))
  {
    return false;
  }

  if (release_pending_) {
    if ((now - release_started_at_).seconds() > release_timeout_) {
      RCLCPP_ERROR(node_->get_logger(), "dock release request timed out");
      release_pending_ = false;
      release_future_ = {};
      return false;
    }
    if (release_future_.valid() &&
      release_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      const auto response = release_future_.get();
      release_pending_ = false;
      release_future_ = {};
      return response && response->success &&
             response->request_id == release_request_id_ &&
             response->session_generation == release_session_ &&
             response->charge_state == 0;
    }
    return false;
  }

  if (!release_client_->service_is_ready()) {
    return false;
  }
  auto request = std::make_shared<ChargeRelease::Request>();
  release_request_id_ = makeRequestId();
  release_session_ = state_.session_generation;
  request->request_id = release_request_id_;
  release_started_at_ = now;
  auto future_and_id = release_client_->async_send_request(request);
  release_future_ = future_and_id.future.share();
  release_pending_ = true;
  RCLCPP_INFO(
    node_->get_logger(), "requested zero-motion charge release id=%s",
    release_request_id_.c_str());
  return false;
}

std::string PatrolBotChargingDock::makeRequestId()
{
  static std::random_device device;
  static std::mt19937_64 engine(device());
  static std::mutex random_mutex;
  uint64_t high;
  uint64_t low;
  {
    std::scoped_lock lock(random_mutex);
    high = engine();
    low = engine();
  }
  high = (high & 0xffffffffffff0fffULL) | 0x0000000000004000ULL;
  low = (low & 0x3fffffffffffffffULL) | 0x8000000000000000ULL;
  std::ostringstream stream;
  stream << std::hex << std::setfill('0')
         << std::setw(8) << static_cast<uint32_t>(high >> 32) << '-'
         << std::setw(4) << static_cast<uint16_t>(high >> 16) << '-'
         << std::setw(4) << static_cast<uint16_t>(high) << '-'
         << std::setw(4) << static_cast<uint16_t>(low >> 48) << '-'
         << std::setw(12) << (low & 0x0000ffffffffffffULL);
  return stream.str();
}

}  // namespace patrolbot_docking_plugins

PLUGINLIB_EXPORT_CLASS(
  patrolbot_docking_plugins::PatrolBotChargingDock,
  opennav_docking_core::ChargingDock)
