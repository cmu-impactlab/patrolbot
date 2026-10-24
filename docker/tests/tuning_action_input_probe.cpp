// Real composed BtNavigator endpoint and SimpleActionServer input admission.
// Fake action callbacks publish no velocity, pose, hardware or motor commands.
#include <atomic>
#include <cassert>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include "nav2_bt_navigator/bt_navigator.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/tuning_action_gate.hpp"
#include "patrolbot_interfaces/srv/tuning_maintenance.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using Spin = nav2_msgs::action::Spin;
using Follow = nav2_msgs::action::FollowPath;
using Maintenance = patrolbot_interfaces::srv::TuningMaintenance;

template<typename Predicate>
void until(Predicate predicate)
{
  const auto deadline = std::chrono::steady_clock::now() + 10s;
  while (!predicate()) {
    assert(std::chrono::steady_clock::now() < deadline);
    std::this_thread::sleep_for(1ms);
  }
}

template<typename Future>
auto result(Future & future)
{
  assert(future.wait_for(10s) == std::future_status::ready);
  return future.get();
}

void races()
{
  nav2_util::TuningActionGate gate;
  gate.core_held = false;
  for (int i = 0; i < 1000; ++i) {
    std::atomic<int> ready{0};
    std::atomic<bool> go{false};
    bool goal = false, held = false;
    nav2_util::TuningActionGate::GoalId uuid{};
    std::thread incoming([&]() {
      ++ready; while (!go.load()) {std::this_thread::yield();}
      goal = gate.beginGoal(&gate, uuid);
    });
    std::thread maintenance([&]() {
      ++ready; while (!go.load()) {std::this_thread::yield();}
      std::lock_guard<std::mutex> lock(gate.mutex);
      if (gate.goals.empty() && gate.workers == 0) {gate.held = held = true;}
    });
    while (ready.load() != 2) {std::this_thread::yield();}
    go.store(true);
    incoming.join(); maintenance.join();
    assert(goal != held);
    if (goal) {gate.endGoal(&gate, uuid);}
    gate.held = false;
    assert(!gate.faulted);
  }
}

int main(int argc, char ** argv)
{
  races();
  rclcpp::init(argc, argv);
  auto client_node = std::make_shared<rclcpp::Node>("action_input_fixture_client");
  auto lease_client = client_node->create_client<Maintenance>("/bt_navigator/tuning_maintenance");
  auto spin_client = rclcpp_action::create_client<Spin>(client_node, "/fixture_spin");
  auto follow_client = rclcpp_action::create_client<Follow>(client_node, "/fixture_follow");
  auto core_hold = client_node->create_publisher<std_msgs::msg::Bool>("/patrolbot/tuning_hold", 10);
  auto navigator = std::make_shared<nav2_bt_navigator::BtNavigator>();
  auto consumers = std::make_shared<rclcpp::Node>("fake_action_consumers");
  std::atomic<bool> finish{false}, tail{false};
  std::atomic<int> spin_calls{0}, follow_calls{0};
  std::unique_ptr<nav2_util::SimpleActionServer<Spin>> spin;
  spin = std::make_unique<nav2_util::SimpleActionServer<Spin>>(
    consumers, "fixture_spin", [&]() {
      ++spin_calls;
      while (!finish.load()) {
        if (spin->is_cancel_requested()) {spin->terminate_current(); return;}
        std::this_thread::sleep_for(1ms);
      }
      spin->succeeded_current();
      while (tail.load()) {std::this_thread::sleep_for(1ms);}
    }, nullptr, 500ms, true);
  std::unique_ptr<nav2_util::SimpleActionServer<Follow>> follow;
  follow = std::make_unique<nav2_util::SimpleActionServer<Follow>>(
    consumers, "fixture_follow", [&]() {++follow_calls; follow->succeeded_current();},
    nullptr, 500ms, true);
  spin->activate(); follow->activate();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(client_node);
  executor.add_node(navigator->get_node_base_interface());
  executor.add_node(consumers);
  std::thread thread([&]() {executor.spin();});
  assert(lease_client->wait_for_service(10s));
  assert(spin_client->wait_for_action_server(10s) && follow_client->wait_for_action_server(10s));
  auto lease = [&](const std::string & operation, const std::string & job = "11111111-1111-4111-8111-111111111111") {
      auto request = std::make_shared<Maintenance::Request>();
      request->operation = operation; request->job_id = job;
      request->digest = std::string(64, 'a'); request->plan_hash = std::string(64, 'b');
      auto future = lease_client->async_send_request(request);
      return result(future);
    };
  auto & gate = nav2_util::tuningActionGate();
  auto idle = [&]() {
      std::lock_guard<std::mutex> lock(gate.mutex);
      return gate.goals.empty() && gate.workers == 0;
    };
  auto hold = [&](bool held) {
      until([&]() {return core_hold->get_subscription_count() == 1;});
      std_msgs::msg::Bool message; message.data = held;
      core_hold->publish(message);
      until([&]() {std::lock_guard<std::mutex> lock(gate.mutex); return gate.core_held == held;});
    };
  assert(!lease("inspect")->accepted);  // Read-only reply-readiness barrier.
  const auto observed = lease("observe");
  assert(observed->accepted && observed->evidence_json.find("composed-actions-v1") != std::string::npos);
  hold(false);

  auto first_future = spin_client->async_send_goal(Spin::Goal());
  auto first = result(first_future); assert(first);
  until([&]() {return spin_calls == 1;});
  auto pending_future = spin_client->async_send_goal(Spin::Goal());
  auto pending = result(pending_future); assert(pending);
  assert(!lease("acquire")->accepted);  // Current AND pending work, never canceled to get ownership.
  auto first_result = spin_client->async_get_result(first);
  auto pending_result = spin_client->async_get_result(pending);
  finish.store(true);
  assert(result(first_result).code == rclcpp_action::ResultCode::SUCCEEDED);
  assert(result(pending_result).code == rclcpp_action::ResultCode::SUCCEEDED);
  until(idle);
  assert(spin_calls == 2);

  tail.store(true);
  auto tail_future = spin_client->async_send_goal(Spin::Goal());
  auto tail_goal = result(tail_future); assert(tail_goal);
  auto tail_result = spin_client->async_get_result(tail_goal);
  assert(result(tail_result).code == rclcpp_action::ResultCode::SUCCEEDED);
  assert(!lease("acquire")->accepted);  // Result sent, but callback still using execution state.
  tail.store(false);
  until(idle);

  assert(lease("acquire")->accepted);
  hold(true);
  const int before = spin_calls.load();
  auto rejected_spin = spin_client->async_send_goal(Spin::Goal());
  auto rejected_follow = follow_client->async_send_goal(Follow::Goal());
  assert(!result(rejected_spin) && !result(rejected_follow));
  assert(!lease("release", "22222222-2222-4222-8222-222222222222")->accepted);
  const auto inspected = lease("inspect");
  assert(inspected->accepted && inspected->evidence_json.find("composed-actions-v1") != std::string::npos);
  assert(lease("release")->accepted);
  auto release_gap = spin_client->async_send_goal(Spin::Goal());
  assert(!result(release_gap));  // Peer release cannot open the core's still-held input boundary.
  hold(false);
  assert(spin_calls == before && follow_calls == 0 && idle());  // No pending/replayed callback.
  auto fresh_spin = spin_client->async_send_goal(Spin::Goal());
  auto fresh_follow = follow_client->async_send_goal(Follow::Goal());
  auto new_spin = result(fresh_spin); auto new_follow = result(fresh_follow);
  assert(new_spin && new_follow);
  auto new_spin_result = spin_client->async_get_result(new_spin);
  auto new_follow_result = follow_client->async_get_result(new_follow);
  assert(result(new_spin_result).code == rclcpp_action::ResultCode::SUCCEEDED);
  assert(result(new_follow_result).code == rclcpp_action::ResultCode::SUCCEEDED);
  until(idle);

  finish.store(false);
  auto cancel_future = spin_client->async_send_goal(Spin::Goal());
  auto cancel_goal = result(cancel_future); assert(cancel_goal);
  hold(true);
  assert(!lease("acquire", "33333333-3333-4333-8333-333333333333")->accepted);
  auto cancellation = spin_client->async_cancel_goal(cancel_goal);
  assert(result(cancellation)->return_code == 0);
  auto canceled_result = spin_client->async_get_result(cancel_goal);
  assert(result(canceled_result).code == rclcpp_action::ResultCode::CANCELED);
  until(idle);
  hold(false);
  assert(!lease("acquire")->accepted);  // A delayed old owner cannot reacquire after release.
  finish.store(true);
  spin->deactivate(); follow->deactivate();
  executor.cancel(); thread.join();
  follow.reset(); spin.reset();
  rclcpp::shutdown();
  std::cout << "PASS: composed direct actions, pending goals, worker tail, release gap, cancel, and 1000 admission races\n";
}
