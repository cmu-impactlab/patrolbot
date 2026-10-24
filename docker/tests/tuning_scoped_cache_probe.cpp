// Native 1.3.12 RPP ParameterHandler, without a controller/costmap/TF prerequisite.
// This runs only in the isolated Docker fixture. It neither publishes nor seeds.
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_regulated_pure_pursuit_controller/parameter_handler.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_cache_probe");
  std::string plugin = "FollowPath";
  auto logger = node->get_logger();
  auto handler = std::make_unique<nav2_regulated_pure_pursuit_controller::ParameterHandler>(
    node, plugin, logger, 6.0);
  // Deliberately offer reconstruction, equivalent to plugin cleanup/configure.
  auto reconstruct = node->create_service<std_srvs::srv::Trigger>(
    "~/reconstruct", [&](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      handler.reset();
      handler = std::make_unique<nav2_regulated_pure_pursuit_controller::ParameterHandler>(
        node, plugin, logger, 6.0);
      response->success = true;
    });
  rclcpp::spin(node->get_node_base_interface());
  handler.reset();
  rclcpp::shutdown();
}
