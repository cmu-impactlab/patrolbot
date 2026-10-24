// Fixed exact-job maintenance for the existing sole velocity mux publisher.
// Included by the pinned twist_mux overlay; not a new velocity producer.
#ifndef PATROLBOT_MUX_MAINTENANCE_GATE_HPP_
#define PATROLBOT_MUX_MAINTENANCE_GATE_HPP_

#include <functional>
#include <iomanip>
#include <map>
#include <mutex>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "patrolbot_interfaces/srv/tuning_maintenance.hpp"

namespace twist_mux
{
class MaintenanceGate
{
public:
  explicit MaintenanceGate(rclcpp::Node * node, std::function<bool()> buffers_zero,
    std::function<bool()> configuration_valid)
  : node_(node), buffers_zero_(buffers_zero), configuration_valid_(configuration_valid)
  {
    service_ = node_->create_service<patrolbot_interfaces::srv::TuningMaintenance>(
      "~/tuning_maintenance", [this](
        const std::shared_ptr<patrolbot_interfaces::srv::TuningMaintenance::Request> request,
        std::shared_ptr<patrolbot_interfaces::srv::TuningMaintenance::Response> response) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        response->accepted = false;
        response->detail = "invalid_request";
        static const std::regex uuid("[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}");
        static const std::regex digest("sha256:[0-9a-f]{64}");
        if (!std::regex_match(request->job_id, uuid) || !std::regex_match(request->digest, digest) ||
          !std::regex_match(request->plan_hash, digest) || !request->payload_json.empty()) {return;}
        const bool owner = job_ == request->job_id && digest_ == request->digest && plan_ == request->plan_hash;
        const bool idle = buffers_zero_();
        const bool graph = graphValid() && configuration_valid_();
        if (request->operation == "acquire") {
          if (owner) {response->accepted = idle && graph;} else if (
            job_.empty() && idle && graph && !retired_.count(request->job_id) && retired_.size() < 1024)
          {
            job_ = request->job_id; digest_ = request->digest; plan_ = request->plan_hash;
            response->accepted = true;
          }
        } else if (request->operation == "inspect") {
          response->accepted = owner && idle && graph;
        } else if (request->operation == "release") {
          if (owner && idle && graph) {
            retired_.insert(job_);
            released_job_ = job_; released_digest_ = digest_; released_plan_ = plan_;
            // Discard any Twist queued before/during maintenance, even if it is
            // delivered after release. A fresh producer action must generate a
            // new middleware sample; old input is never replayed from mux cache.
            released_at_ = rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds();
            job_.clear(); digest_.clear(); plan_.clear();
            response->accepted = true;
          } else if (job_.empty()) {
            response->accepted = released_job_ == request->job_id &&
              released_digest_ == request->digest && released_plan_ == request->plan_hash;
          }
        }
        response->detail = response->accepted ? "mux_maintenance_acknowledged" : "mux_maintenance_busy";
        std::ostringstream out;
        out << "{\"version\":2,\"implementation\":\"twist-mux-4.5.0-maintenance-v1\",\"owner\":";
        if (job_.empty()) {out << "null";} else {
          out << "{\"job_id\":" << std::quoted(job_) << ",\"digest\":" << std::quoted(digest_)
              << ",\"plan_hash\":" << std::quoted(plan_) << '}';
        }
        out << ",\"idle\":" << (idle ? "true" : "false")
            << ",\"command_buffers_zero\":" << (idle ? "true" : "false")
            << ",\"fixed_graph_verified\":" << (graph ? "true" : "false")
            << ",\"all_mux_inputs_held\":" << (job_.empty() ? "false" : "true") << '}';
        response->evidence_json = out.str();
      });
  }

  std::recursive_mutex & mutex() {return mutex_;}
  bool accept(const rclcpp::MessageInfo & info) const
  {
    if (!job_.empty()) {return false;}
    return released_at_ == 0 || info.get_rmw_message_info().source_timestamp > released_at_;
  }
  bool held() const {return !job_.empty();}

private:
  bool graphValid() const
  {
    // No publisher is accepted as a release signal. The switch path is explicitly
    // disabled by this mux's held callback gate and must have an empty cache and
    // zero publishers; other paths require exactly their known producer.
    static const std::map<std::string, std::string> publishers{
      {"/input/safety_controller", "patrolbot_safety_watchdog"},
      {"/input/undock_turn", "patrolbot_bridge"}, {"/input/joy", "patrolbot_joy_teleop"},
      {"/input/teleop", "patrolbot_web_bridge"}, {"/input/navi", "collision_monitor"},
      {"/input/switch", ""}};
    for (const auto & entry : publishers) {
      const auto endpoints = node_->get_publishers_info_by_topic(entry.first);
      if (entry.second.empty()) {if (!endpoints.empty()) {return false;}} else if (
        endpoints.size() != 1 || endpoints[0].node_name() != entry.second ||
        endpoints[0].node_namespace() != "/" || endpoints[0].topic_type() != "geometry_msgs/msg/Twist")
      {return false;}
    }
    return true;
  }
  rclcpp::Node * node_;
  std::function<bool()> buffers_zero_, configuration_valid_;
  std::recursive_mutex mutex_;
  std::string job_, digest_, plan_, released_job_, released_digest_, released_plan_;
  std::set<std::string> retired_;
  int64_t released_at_{0};
  rclcpp::Service<patrolbot_interfaces::srv::TuningMaintenance>::SharedPtr service_;
};
}  // namespace twist_mux
#endif
