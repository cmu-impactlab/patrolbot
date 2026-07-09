#ifndef ROSARIA2_INCLUDE_ROSARIA2_LASERPUBLISHER_HPP_
#define ROSARIA2_INCLUDE_ROSARIA2_LASERPUBLISHER_HPP_

#include <string>
#ifdef ADEPT_PKG
  #include <Aria.h>
#else
  #include <Aria/Aria.h>
#endif
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>

// why forwrd declarations?
class ArLaser;
class ArTime;





//------------------------------------------------------------------------------
/// @brief      Class that warps around a Node instancen ArLaser instance and publishe
///
class LaserPublisher {
 public:
    LaserPublisher(ArLaser* _l, rclcpp::Node& _n, bool _broadcast_transform = true, const std::string& _tf_frame = "laser", const std::string& _parent_tf_frame = "base_link", const std::string& _global_tf_frame = "odom");
    ~LaserPublisher();

 protected:
    void readings_cb();
    void publish_laser_scan();
    void publish_point_cloud();

    rclcpp::Node& node;
    ArLaser *laser;

    // callback (on new )
    ArFunctorC< LaserPublisher > laser_readings_cb;

    // publishers
    rclcpp::Publisher< sensor_msgs::msg::LaserScan >::SharedPtr laserscan_pub;
    rclcpp::Publisher< sensor_msgs::msg::PointCloud >::SharedPtr pointcloud_pub;

    // message buffers
    sensor_msgs::msg::LaserScan laserscan;
    sensor_msgs::msg::PointCloud pointcloud;

    // frame names
    std::string tfname;
    std::string parenttfname;
    std::string globaltfname;

    // transform buffer
    tf2::Transform lasertf;

    // broadcaster and on/off flag
    std::unique_ptr< tf2_ros::TransformBroadcaster > transform_broadcaster;
    bool broadcast_tf;

    //ArTime *readingsCallbackTime;
};


#endif  // ROSARIA2_INCLUDE_ROSARIA2_LASERPUBLISHER_HPP_