#include <rclcpp/rclcpp.hpp>
#include "rosaria2/rosaria2_node.hpp"

int main(int argc, char const *argv[]) {

    Aria::init();

    rclcpp::init(argc, argv);
    auto rosaria_node = std::make_shared< RosAria2Node >();

    if (rosaria_node->setup() != 0) {
        RCLCPP_FATAL(rosaria_node->get_logger(), "RosAria: ROS node setup failed... \n" );
        return -1;
    }

    rclcpp::spin(rosaria_node);

    Aria::shutdown();

    return 0;
}
