


#include <math.h>
#include <boost/algorithm/string.hpp>
#ifdef ADEPT_PKG
  #include <Aria.h>
  #include <ArRobotConfigPacketReader.h>        // @todo remove after ArRobotConfig implemented in AriaCoda
#else
  #include <Aria/Aria.h>
  #include <Aria/ArRobotConfigPacketReader.h>   // @todo remove after ArRobotConfig implemented in AriaCoda
#endif
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "rosaria2/artime_to_ros_time.hpp"
#include "rosaria2/laser_publisher.hpp"

// TODO publish pointcloud of cumulative readings in separate topic?
// TODO generic pointcloud sensor publisher (seprate point cloud stuff there)
// TODO make  similar sonar publisher?

LaserPublisher::LaserPublisher(ArLaser *_l, rclcpp::Node& _n, bool _broadcast_tf, const std::string& _tf_frame, const std::string& _parent_tf_frame, const std::string& _global_tf_frame) :
    laser_readings_cb(this, &LaserPublisher::readings_cb),
    node(_n),
    laser(_l),
    tfname(_tf_frame),
    parenttfname(_parent_tf_frame),
    globaltfname(_global_tf_frame),
    broadcast_tf(_broadcast_tf) {
        assert(_l);
        laser->lockDevice();
        laser->addReadingCB(&laser_readings_cb);
        laser->unlockDevice();
        std::string laserscan_name(laser->getName());
        boost::erase_all(laserscan_name,".");
        laserscan_name += "_laserscan";
        std::string pointcloud_name(laser->getName());
        boost::erase_all(pointcloud_name,".");
        pointcloud_name += "_pointcloud";
        laserscan_pub = node.create_publisher< sensor_msgs::msg::LaserScan >(laserscan_name, 20);
        pointcloud_pub = node.create_publisher< sensor_msgs::msg::PointCloud >(pointcloud_name, 50);
        transform_broadcaster = std::make_unique< tf2_ros::TransformBroadcaster >(node);

        tf2::Quaternion q;
        if(laser->hasSensorPosition()) {
            lasertf.setOrigin(tf2::Vector3(laser->getSensorPositionX() / 1000.0, laser->getSensorPositionY() / 1000.0, laser->getSensorPositionZ() / 1000.0));
            q.setRPY(0, 0, ArMath::degToRad(laser->getSensorPositionTh()));
        } else {
            lasertf.setOrigin(tf2::Vector3(0, 0, 0));
            q.setRPY(0, 0, 0);
        }
        lasertf.setRotation(q);

        laserscan.header.frame_id = "laser_frame";
        laserscan.angle_min = ArMath::degToRad(laser->getStartDegrees());
        laserscan.angle_max = ArMath::degToRad(laser->getEndDegrees());
        //laserscan.time_increment = ?
        laserscan.range_min = 0; //laser->getMinRange() / 1000.0;
        laserscan.range_max = laser->getMaxRange() / 1000.0;
        pointcloud.header.frame_id = globaltfname;

        // Get angle_increment of the laser
        laserscan.angle_increment = 0;
        if(laser->canSetIncrement()) {
            laserscan.angle_increment = laser->getIncrement();
        } else if(laser->getIncrementChoice() != NULL) {
            laserscan.angle_increment = laser->getIncrementChoiceDouble();
        }
        assert(laserscan.angle_increment > 0);
        laserscan.angle_increment *= M_PI / 180.0;

        //readingsCallbackTime = new ArTime;
}


LaserPublisher::~LaserPublisher() {
    laser->lockDevice();
    laser->remReadingCB(&laser_readings_cb);
    laser->unlockDevice();
    //delete readingsCallbackTime;
}


void LaserPublisher::readings_cb() {
    //printf("readingsCB(): %lu ms since last readingsCB() call.\n", readingsCallbackTime->mSecSince());
    assert(laser);
    laser->lockDevice();
    publish_laser_scan();
    publish_point_cloud();
    laser->unlockDevice();
    if (broadcast_tf) {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.frame_id = parenttfname;
        tf.header.stamp = toROSTime(laser->getLastReadingTime(), node);
        tf.child_frame_id = tfname;
        tf.transform = tf2::toMsg(lasertf);
        transform_broadcaster->sendTransform(tf);
        // transform_broadcaster->sendTransform(tf2::StampedTransform(lasertf, convertArTimeToROS(laser->getLastReadingTime()), parenttfname, tfname));
    }
    //readingsCallbackTime->setToNow();
}


void LaserPublisher::publish_laser_scan() {
    // laserscan.header.stamp = convertArTimeToROS(laser->getLastReadingTime());
    laserscan.header.stamp = toROSTime(laser->getLastReadingTime(), node);
    const std::list< ArSensorReading* > *readings = laser->getRawReadings();
    assert(readings);
    //printf("laserscan: %lu readings\n", readings->size());
    laserscan.ranges.resize(readings->size());
    size_t n = 0;
    if (laser->getFlipped()) {
        // Reverse the data
        for (std::list< ArSensorReading* >::const_reverse_iterator r = readings->rbegin(); r != readings->rend(); ++r) {
            assert(*r);
            if ((*r)->getIgnoreThisReading()) {
                laserscan.ranges[n] = -1;
            } else {
                laserscan.ranges[n] = (*r)->getRange() / 1000.0;
            }
            ++n;
        }
    } else {
        for (std::list< ArSensorReading* >::const_iterator r = readings->begin(); r != readings->end(); ++r) {
            assert(*r);
            if ((*r)->getIgnoreThisReading()) {
                laserscan.ranges[n] = -1;
            } else {
                laserscan.ranges[n] = (*r)->getRange() / 1000.0;
            }
            ++n;
        }
    }

    laserscan_pub->publish(laserscan);
}


void LaserPublisher::publish_point_cloud() {
    assert(laser);
    // pointcloud.header.stamp = convertArTimeToROS(laser->getLastReadingTime());
    pointcloud.header.stamp = toROSTime(laser->getLastReadingTime(), node);

    #ifdef ARIACODA
    const std::list< ArPoseWithTime > p = laser->getCurrentRangeBuffer().getBuffer();
    pointcloud.points.resize(p.size());
    size_t n = 0;
    for (auto i = p.cbegin(); i != p.cend(); ++i) {
        pointcloud.points[n].x = i->getX() / 1000.0;
        pointcloud.points[n].y = i->getY() / 1000.0;
        pointcloud.points[n].z = (laser->hasSensorPosition() ?  laser->getSensorPositionZ() / 1000.0 : 0.0);
            // XXX TODO ^--- is this correct, or should laser position on robot be
            // reflected in tf for laser (from URDF or from Aria params if not
            // available?) and the height of sensor readings determined by ROS
            // client using that?
        ++n;
    }
    #else
    assert(laser->getCurrentRangeBuffer());
    const std::list< ArPoseWithTime* >* p = laser->getCurrentRangeBuffer()->getBuffer();
    pointcloud.points.resize(p->size());
    size_t n = 0;
    for (auto i = p->cbegin(); i != p->cend(); ++i) {
        pointcloud.points[n].x = (*i)->getX() / 1000.0;
        pointcloud.points[n].y = (*i)->getY() / 1000.0;
        pointcloud.points[n].z = (laser->hasSensorPosition() ?  laser->getSensorPositionZ() / 1000.0 : 0.0);
            // XXX TODO ^--- is this correct, or should laser position on robot be
            // reflected in tf for laser (from URDF or from Aria params if not
            // available?) and the height of sensor readings determined by ROS client
            // using that?
        ++n;
    }
    #endif

    pointcloud_pub->publish(pointcloud);
}
