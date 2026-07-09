#include <Aria/ArRobot.h>
#include <string>
#include <chrono>
#include <sstream>
#ifdef ADEPT_PKG
  #include <Aria.h>
  #include <ArRobotConfigPacketReader.h>        // @todo remove after ArRobotConfig implemented in AriaCoda
#else
  #include <Aria/Aria.h>
  #include <Aria/ArRobotConfigPacketReader.h>   // @todo remove after ArRobotConfig implemented in AriaCoda
#endif
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/empty.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>           // for sonar data
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>    // can optionally publish sonar as new type pointcloud2
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>              // for tf::getPrefixParam
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rosaria2/rosaria2_node.hpp"


using namespace std::chrono_literals;  // for ""ms literal
using namespace std::placeholders;     // for _1, _2

#ifndef ROSARIA2_DEFAULT_CMDVEL_TIMEOUT
#define ROSARIA2_DEFAULT_CMDVEL_TIMEOUT 600ms  // cf. rosaria package (legacy)
#endif


RosAria2Node::Parameters::Parameters(rclcpp::Node* node) :
    _param_subscriber(std::make_shared< rclcpp::ParameterEventHandler >(node)),
    serial_port(node, "seria_port", "/dev/ttyUSB0"),
    serial_baud(node, "serial_baud", 9600),
    sonar_enabled(node, "sonar_enabled", false),
    publish_sonar(node, "publish_sonar", false),
    publish_sonar_pointcloud2(node, "publish_sonar_pointcloud2", false),
    publish_aria_lasers(node, "publish_aria_lasers", false),
    publish_motors_state(node, "publish_motors_state", false),
    debug_aria(node, "debug_aria", false),
    aria_log_filename(node, "aria_log_filename", "Aria.log"),
    ticks_mm(node, "ticks_mm", -1),
    drift_factor(node, "drif_factor", -1),
    rev_count(node, "rev_count", -1) {
        /* ... */

        // parameters are declared on ROS parameter server; pre-set values on parameter server are preserved/override default values
}


RosAria2Node::RosAria2Node(const std::string& name) :
    rclcpp::Node(name),
    // runtime configuration handler
    config(std::make_shared< RosAria2Node::Parameters >(this)),
    // control timeout (max delay between vel commands)
    cmdvel_timeout(ROSARIA2_DEFAULT_CMDVEL_TIMEOUT),
    // ...
    publish_cb(this, &RosAria2Node::publish) {
        // @todo check if Aria is running, initialize otherwise:
        //   Aria::init();

        // avertise publishers
        // @note about latching in ROS2 cf. https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html#comparison-to-ros-1
        pose_pub = this->create_publisher< nav_msgs::msg::Odometry >("pose", 1000);
        bumpers_pub = this->create_publisher< rosaria2::msg::BumperState >("bumper_state", 1000);
        sonar_pub = this->create_publisher< sensor_msgs::msg::PointCloud >("sonar", 50);
        sonar_pointcloud2_pub = this->create_publisher< sensor_msgs::msg::PointCloud2 >("sonar_pointcloud2", 50);
        voltage_pub = this->create_publisher< std_msgs::msg::Float64 >("battery_voltage", 1000);
        recharge_state_pub = this->create_publisher< std_msgs::msg::Int8 >("battery_recharge_state", rclcpp::QoS(5).best_effort().transient_local());
        state_of_charge_pub = this->create_publisher< std_msgs::msg::Float32 >("battery_state_of_charge", 100);
        motors_state_pub = this->create_publisher< std_msgs::msg::Bool >("motors_state", rclcpp::QoS(5).best_effort().transient_local());

        // advertise enable/disable services
        enable_srv = this->create_service< std_srvs::srv::Empty >("enable_motors", std::bind(&RosAria2Node::enable_motors_cb, this, _1, _2), 10);
        disable_srv = this->create_service< std_srvs::srv::Empty >("disable_motors", std::bind(&RosAria2Node::disable_motors_cb, this, _1, _2), 10);

        // initialize transform broadcaster
        odom_broadcaster = std::make_unique< tf2_ros::TransformBroadcaster >(*this);

        // set initial state for (internal) message buffers
        motors_state.data = false;

        // backup current time
        veltime = this->now();
}


RosAria2Node::~RosAria2Node() {
    // disable motors and sonar.
    robot->disableMotors();
    robot->disableSonar();
    robot->stopRunning();
    robot->waitForRunExit();
    Aria::shutdown();
    RCLCPP_INFO(this->get_logger(), "shutdown!");
}


int RosAria2Node::setup() {
    // Note, various objects are allocated here which are never deleted (freed), since Setup() is only supposed to be
    // called once per instance, and these objects need to persist until the process terminates. (won't it survive *after* the process terminates though?)
    // @todo refactor to use std::shared_ptr
    robot = std::make_shared< ArRobot >();                               // should be instantiated on costruction of class
    auto args = std::make_shared< ArArgumentBuilder >();                 // no need to be pointers, only used locally, but could be smart pointer members to ensure required lifetime //  never freed
    auto argparser = std::make_shared< ArArgumentParser >(args.get());   // no need to be pointers, only used locally, but could be smart pointer members to ensure required lifetime // Warning never freed

    // adds any arguments given in /etc/Aria.args.
    // @note    useful on robots with unusual serial port or baud rate (e.g. pioneer lx)
    argparser->loadDefaultArguments();

    // add parameters given via ROS params (see RosAriaNode constructor):
    // @todo move to Parameters::parse() to simplify setup(), as this is not specific to robot initialization

    // if serial port parameter contains a ':' character, then interpret it as hostname:tcpport
    // for wireless serial connection. Otherwise, interpret it as a serial port name.
    size_t colon_pos = config->serial_port.get().find(":");
    if (colon_pos != std::string::npos) {
        args->add("-remoteHost"); // pass robot's hostname/IP address to Aria
        args->add(config->serial_port.get().substr(0, colon_pos).c_str());
        args->add("-remoteRobotTcpPort"); // pass robot's TCP port to Aria
        args->add(config->serial_port.get().substr(colon_pos + 1).c_str());
    } else {
        args->add("-robotPort %s", config->serial_port.get().c_str()); // pass robot's serial port to Aria
    }

    // if a baud rate was specified in baud parameter
    if (config->serial_baud != 0) {
        args->add("-robotBaud %d", config->serial_baud.get());
    }

    // turn on all ARIA debugging
    if (config->debug_aria) {
        args->add("-robotLogPacketsReceived"); // log received packets
        args->add("-robotLogPacketsSent"); // log sent packets
        args->add("-robotLogVelocitiesReceived"); // log received velocities
        args->add("-robotLogMovementSent");
        args->add("-robotLogMovementReceived");
        ArLog::init(ArLog::File, ArLog::Verbose, config->aria_log_filename.get().c_str(), true);
    }

    // connect to the robot
    conn = std::make_shared< ArRobotConnector >(argparser.get(), robot.get()); // warning never freed
    if (!conn->connectRobot()) {
        RCLCPP_ERROR(this->get_logger(), "Aria could not connect to robot! (Check ~port parameter is correct, and permissions on port device, or any errors reported above)");
        return 1;
    }

    // create laser connection (when configured)
    if(config->publish_aria_lasers) {
        laserConnector = std::make_shared< ArLaserConnector >(argparser.get(), robot.get(), conn.get());
    }

    // causes ARIA to load various robot-specific hardware parameters from the robot parameter file in /usr/local/Aria/params
    // @note    not to be confused with command line arguments parsed above
    if (!Aria::parseArgs()) {
        RCLCPP_ERROR(this->get_logger(), "Aria error parsing startup parameters!");
        return 1;
    }

    // read_parameters(); // reads TicksMM, DriftFactor and Revcount as dynamic_reconfigure parameters

    // enable the motors
    robot->enableMotors();

    // disable sonars on startup
    robot->disableSonar();

    // callback will  be called by ArRobot background processing thread for every SIP data packet received from robot
    robot->addSensorInterpTask("ROSPublishingTask", 100, &publish_cb);

    // initialize bumpers with robot number of bumpers
    bumpers.front_bumpers.resize(robot->getNumFrontBumpers());
    bumpers.rear_bumpers.resize(robot->getNumRearBumpers());

    // run ArRobot background processing thread
    robot->runAsync(true);

    // connect to lasers and create publishers
    if(config->publish_aria_lasers) {
        RCLCPP_INFO(this->get_logger(), "Connecting to laser(s) configured in ARIA parameter file(s)...");

        // laser connection logic, performs connection and instantiates a LaserPublisher instance
        if (!laserConnector->connectLasers()) {
            RCLCPP_ERROR(this->get_logger(), "Error connecting to laser(s)...");
            return 1;
        }

        robot->lock();
        const std::map< int, ArLaser* > *lasers = robot->getLaserMap();
        RCLCPP_INFO(this->get_logger(), "there are %lu connected lasers", lasers->size());
        for (std::map<int, ArLaser*>::const_iterator i = lasers->begin(); i != lasers->end(); ++i) {
            ArLaser *l = i->second;
            int ln = i->first;
            std::string tfname("laser");
            if (lasers->size() > 1 || ln > 1) {
                // no number if only one laser which is also laser 1
                tfname += ln;
            }
            tfname += "_frame";
            RCLCPP_INFO(this->get_logger(), "Creating publisher for laser #%d named %s with tf frame name %s", ln, l->getName(), tfname.c_str());
            new LaserPublisher(l, *this, true, tfname);
        }
        robot->unlock();
        RCLCPP_INFO(this->get_logger(), "Done creating laser publishers");
    }

    // subscribe to command topics
    cmdvel_sub = this->create_subscription< geometry_msgs::msg::Twist >("cmd_vel", 10, std::bind(&RosAria2Node::cmdvel_cb, this, _1));

    // register a watchdog for cmd_vel timeout
    // @note null timeout disables timer
    size_t cmdvel_timeout_param = 0;
    this->get_parameter_or< size_t >("cmd_vel_timeout", cmdvel_timeout_param, 600 /* ms */);
    cmdvel_timeout = rclcpp::Duration(std::chrono::milliseconds(cmdvel_timeout_param));
    if (cmdvel_timeout_param > 0.0) {
        // @todo expose timer frequency as a parameter?
        // @todo check if wall timer vs timer for watchdog
        cmdvel_watchdog_timer = this->create_timer(100ms /* no need to wrap around a Duration instance*/, std::bind(&RosAria2Node::cmdvel_watchdog, this));
    }

    RCLCPP_INFO(this->get_logger(), "Setup complete");
    return 0;
}


void RosAria2Node::cmdvel_cb(const geometry_msgs::msg::Twist& twist) {
    veltime = this->now();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "new speed: [%0.2f,%0.2f](%0.3f)", twist.linear.x*1e3, twist.angular.z, veltime.seconds() );

    robot->lock();
    robot->setVel(twist.linear.x * 1e3);
    if(robot->hasLatVel()) {
        robot->setLatVel(twist.linear.y * 1e3);
    }
    robot->setRotVel(twist.angular.z * 180 / M_PI);
    robot->unlock();

    RCLCPP_DEBUG(this->get_logger(), "sent vels to to aria (time %f): x vel %f mm/s, y vel %f mm/s, ang vel %f deg/s",
        veltime.seconds(), (double) twist.linear.x * 1e3, (double) twist.linear.y * 1e3, (double) twist.angular.z * 180 / M_PI);
}


void RosAria2Node::cmdvel_watchdog() {
    // stop robot if no cmd_vel message was received for *cmdvel_timeout* seconds
    // @todo    espose timeout as a dynamic parameter (!)
    if ((this->now() - veltime) > rclcpp::Duration(600ms)) {
        robot->lock();
        robot->setVel(0.0);
        if(robot->hasLatVel()) {
            robot->setLatVel(0.0);
        }
        robot->setRotVel(0.0);
        robot->unlock();

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "robot stopped");
    }
}


void RosAria2Node::publish() {
    // Note, this is called via SensorInterpTask callback (publish_cb, named "ROSPublishingTask"). ArRobot object 'robot' should not be locked or unlocked.
    // @note pose -> ArPose
    pose = robot->getPose();

    // ArPose::getTh() -> gets the heading (Euler rotation), getThRad could be used instead
    // tf2::Transform::Transform(const Quaternion & q, const Vector3 &)
    tf2::Quaternion heading;
    heading.setRPY(0.0, 0.0, pose.getThRad());  // or pose.getTh() * M_PI / 180
    auto tf = tf2::Transform(heading /* <- heading (yaw) */, tf2::Vector3(pose.getX() / 1000, pose.getY() / 1000, 0) /* <- coords */);  // @note Aria returns pose in mm.

    // -----------------------------------------------------
    // convert transform to ROS2 message for publishing
    // @note position.pose.pose -> geometry_msgs::msg::Pose
    // @note toMsg() populates message in-place (no return value overload implemented in tf2_geometry_msgs)
    tf2::toMsg(tf, position.pose.pose);
    position.twist.twist.linear.x = robot->getVel() / 1000.0; //Aria returns velocity in mm/s.
    position.twist.twist.linear.y = robot->getLatVel() / 1000.0;
    position.twist.twist.angular.z = robot->getRotVel() * M_PI / 180;
    position.header.frame_id = frame_id_odom;
    position.child_frame_id = frame_id_base_link;
    position.header.stamp = this->now();
    pose_pub->publish(position);
    RCLCPP_DEBUG(this->get_logger(), "publish (time %f) pose x: %f, pose y: %f, pose angle: %f; linear vel x: %f, vel y: %f; angular vel z: %f",
        position.header.stamp.nanosec / 1e-9,
        (double)position.pose.pose.position.x,
        (double)position.pose.pose.position.y,
        (double)position.pose.pose.orientation.w,
        (double)position.twist.twist.linear.x,
        (double)position.twist.twist.linear.y,
        (double)position.twist.twist.angular.z
    );

    // -----------------------------------------------------
    // publish base transform (odom->base_link)
    odom_trans.header.stamp = this->now();
    odom_trans.header.frame_id = frame_id_odom;
    odom_trans.child_frame_id = frame_id_base_link;
    odom_trans.transform.translation.x = pose.getX() / 1000.0;
    odom_trans.transform.translation.y = pose.getY() / 1000.0;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf2::toMsg(heading);
    odom_broadcaster->sendTransform(odom_trans);

    // -----------------------------------------------------
    // publish bumper state
    // @note getStallValue returns 2 bytes with stall bit and bumper bits, packed as (00 00 FrontBumpers RearBumpers)
    int stall = robot->getStallValue();
    unsigned char front_bumpers = (unsigned char)(stall >> 8);
    unsigned char rear_bumpers = (unsigned char)(stall);

    bumpers.header.frame_id = frame_id_bumper;
    bumpers.header.stamp = this->now();

    std::stringstream bumper_info(std::stringstream::out);
    // Bit 0 is for stall, next bits are for bumpers (leftmost is LSB)
    for (unsigned int i=0; i<robot->getNumFrontBumpers(); i++) {
        bumpers.front_bumpers[i] = (front_bumpers & (1 << (i+1))) == 0 ? 0 : 1;
        bumper_info << " " << (front_bumpers & (1 << (i+1)));
    }
    RCLCPP_DEBUG(this->get_logger(), "front bumpers:%s", bumper_info.str().c_str());

    bumper_info.str("");
    // Rear bumpers have reverse order (rightmost is LSB)
    unsigned int numRearBumpers = robot->getNumRearBumpers();
    for (unsigned int i=0; i<numRearBumpers; i++) {
        bumpers.rear_bumpers[i] = (rear_bumpers & (1 << (numRearBumpers-i))) == 0 ? 0 : 1;
        bumper_info << " " << (rear_bumpers & (1 << (numRearBumpers-i)));
    }
    RCLCPP_DEBUG(this->get_logger(), "rear bumpers:%s", bumper_info.str().c_str());
    bumpers_pub->publish(bumpers);

    // -----------------------------------------------------
    // publish battery information
    // @todo decide if BatteryVoltageNow (normalized to (0,12)V) is a better option
    std_msgs::msg::Float64 batteryVoltage;
    batteryVoltage.data = robot->getRealBatteryVoltageNow();
    voltage_pub->publish(batteryVoltage);

    if (robot->haveStateOfCharge()) {
        std_msgs::msg::Float32 soc;
        soc.data = robot->getStateOfCharge()/100.0;
        state_of_charge_pub->publish(soc);
    }

    // -----------------------------------------------------
    // publish recharge state if changed
    char s = robot->getChargeState();
    if (s != recharge_state.data) {
        RCLCPP_INFO(this->get_logger(), "publishing new recharge state %d.", s);
        recharge_state.data = s;
        recharge_state_pub->publish(recharge_state);
    }

    // -----------------------------------------------------
    // publish motors state if changed
    bool e = robot->areMotorsEnabled();
    if (e != motors_state.data || !config->publish_motors_state) {
        RCLCPP_INFO(this->get_logger(), "publishing new motors state %d.", e);
        motors_state.data = e;
        motors_state_pub->publish(motors_state);
        config->publish_motors_state = true;
    }

    // -----------------------------------------------------
    // Publish sonar information, if enabled
    // @todo publish only PointCloud2 (PointCloud has been deprecated)
    if (config->publish_sonar || config->publish_sonar_pointcloud2) {
        sensor_msgs::msg::PointCloud cloud;  //sonar readings.
        cloud.header.stamp = position.header.stamp; //copy time.
        // sonar sensors relative to base_link
        cloud.header.frame_id = frame_id_sonar;

        std::stringstream sonar_debug_info; // Log debugging info
        sonar_debug_info << "Sonar readings: ";

        for (int i = 0; i < robot->getNumSonar(); i++) {
            ArSensorReading* reading = NULL;
            reading = robot->getSonarReading(i);
            if (!reading) {
                RCLCPP_WARN(this->get_logger(), "Did not receive a sonar reading.");
                continue;
            }

            // getRange() will return an integer between 0 and 5000 (5m)
            sonar_debug_info << reading->getRange() << " ";

            // local (x,y). Appears to be from the centre of the robot, since values may
            // exceed 5000. This is good, since it means we only need 1 transform.
            // x & y seem to be swapped though, i.e. if the robot is driving north
            // x is north/south and y is east/west.
            //
            //ArPose sensor = reading->getSensorPosition();  //position of sensor.
            // sonar_debug_info << "(" << reading->getLocalX()
            //                  << ", " << reading->getLocalY()
            //                  << ") from (" << sensor.getX() << ", "
            //                  << sensor.getY() << ") ;; " ;

            //add sonar readings (robot-local coordinate frame) to cloud
            geometry_msgs::msg::Point32 p;
            p.x = reading->getLocalX() / 1000.0;
            p.y = reading->getLocalY() / 1000.0;
            p.z = 0.0;
            cloud.points.push_back(p);
        }
        RCLCPP_DEBUG(this->get_logger(), sonar_debug_info.str().c_str());

        // publish topic(s)
        if (config->publish_sonar_pointcloud2) {
            sensor_msgs::msg::PointCloud2 cloud2;
            if (!sensor_msgs::convertPointCloudToPointCloud2(cloud, cloud2)) {
                RCLCPP_WARN(this->get_logger(), "Error converting sonar point cloud message to point_cloud2 type before publishing! Not publishing this time.");
            } else {
                sonar_pointcloud2_pub->publish(cloud2);
            }
        }

        if (config->publish_sonar) {
            sonar_pub->publish(cloud);
        }
    }  // end if publish_sonar || publish_sonar_pointcloud2
}


void RosAria2Node::sonar_connect_cb() {
    config->publish_sonar = (sonar_pub->get_subscription_count() > 0);
    config->publish_sonar_pointcloud2 = (sonar_pointcloud2_pub->get_subscription_count() > 0);
    robot->lock();
    if (config->publish_sonar || config->publish_sonar_pointcloud2) {
        robot->enableSonar();
        config->sonar_enabled = false;
    } else if (!config->publish_sonar && !config->publish_sonar_pointcloud2) {
        robot->disableSonar();
        config->sonar_enabled = true;
    }
    robot->unlock();
}


// void RosAria2Node::read_parameters() {
//     // Robot Parameters. If a parameter was given and is nonzero, set it now.
//     // Otherwise, get default value for this robot (from getOrigRobotConfig()).
//     // Parameter values are stored in member variables for possible later use by the user with dynamic reconfigure.
//     robot->lock();
//     if (this->get_parameter("TicksMM", TicksMM) && TicksMM > 0) {
//         RCLCPP_INFO(this->get_logger(), "Setting robot TicksMM from ROS Parameter: %d", TicksMM);
//         robot->comInt(93, TicksMM);
//     } else {
//         TicksMM = robot->getOrigRobotConfig()->getTicksMM();
//         RCLCPP_INFO(this->get_logger(), "This robot's TicksMM parameter: %d", TicksMM);
//         // this->set_parameter( "TicksMM", TicksMM);
//     }

//     if (this->get_parameter("DriftFactor", DriftFactor) && DriftFactor != -99999) {
//         RCLCPP_INFO(this->get_logger(), "Setting robot DriftFactor from ROS Parameter: %d", DriftFactor);
//         robot->comInt(89, DriftFactor);
//     } else {
//         DriftFactor = robot->getOrigRobotConfig()->getDriftFactor();
//         RCLCPP_INFO(this->get_logger(), "This robot's DriftFactor parameter: %d", DriftFactor);
//         // this->set_parameter( "DriftFactor", DriftFactor);
//     }

//     if (this->get_parameter("RevCount", RevCount) && RevCount > 0) {
//         RCLCPP_INFO(this->get_logger(), "Setting robot RevCount from ROS Parameter: %d", RevCount);
//         robot->comInt(88, RevCount);
//     } else {
//         RevCount = robot->getOrigRobotConfig()->getRevCount();
//         RCLCPP_INFO(this->get_logger(), "This robot's RevCount parameter: %d", RevCount);
//         // this->set_parameter( "RevCount", RevCount);
//     }
//     robot->unlock();
// }


bool RosAria2Node::enable_motors_cb(const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr) {
    RCLCPP_INFO(this->get_logger(), "Enable motors request.");
    robot->lock();
    if(robot->isEStopPressed()) {
        RCLCPP_WARN(this->get_logger(), "RosAria: Warning: Enable motors requested, but robot also has E-Stop button pressed. Motors will not enable.");
    }
    robot->enableMotors();
    robot->unlock();
    // todo could wait and see if motors do become enabled, and send a response with an error flag if not
    return true;
}


bool RosAria2Node::disable_motors_cb(const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr) {
    RCLCPP_INFO(this->get_logger(), "Disable motors request.");
    robot->lock();
    robot->disableMotors();
    robot->unlock();
    // todo could wait and see if motors do become disabled, and send a response with an error flag if not
    return true;
}
