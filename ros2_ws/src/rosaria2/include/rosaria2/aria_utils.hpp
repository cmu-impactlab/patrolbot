#ifndef ROSARIA2_INCLUDE_ROSARIA2_ARIAUTILS_HPP_
#define ROSARIA2_INCLUDE_ROSARIA2_ARIAUTILS_HPP_


#include <memory>



namespace aria {



#include <rclcpp/rclcpp.hpp>
#ifdef ADEPT_PKG
#include "ariaUtil.h"
#else
#include "Aria/ariaUtil.h"
#endif


//------------------------------------------------------------------------------
/// @brief      Converts ArTime instances to ROS2's rclcpp::Time type.
///
/// @param[in]  t     ArTime instance to convert.
/// @param[in]  node  ROS2 node to fetch time reference.
///
/// @return     Instance of rclcpp::Time.
///
rclcpp::Time toROSTime(const ArTime& t, const rclcpp::Node& node) {
    // ARIA/ARNL times are in reference to an arbitrary starting time, not OS clock, so find the time elapsed between now and t
    // to adjust the time stamp in ROS time vs. now accordingly.
    ArTime arianow;
    const double dtsec = (double) t.mSecSince(arianow) / 1000.0;
    //printf("was %f seconds ago\n", dtsec);
    return rclcpp::Time(node.now().seconds() - dtsec);
}



//------------------------------------------------------------------------------
/// @brief      Aria wrapper class implementing robot communication/control.
///             Restricts calls to Aria/AriaCoda to class definition, useful in modular design of ROS applications.
///
class RobotHandle {
 public:
    //--------------------------------------------------------------------------
    /// @brief      Pointer type alias
    ///
    using Ptr = std::shared_ptr< RobotHandle >;

    RobotHandle();
    ~RobotHandle();



 protected:
    std::shared_ptr< ArRobot > _robot;
    std::shared_ptr< ArRobotConnector > _rc;
    std::shared_ptr< ArLaserConnector > _lc;

    // @todo remove, unlikely to be needed post-initialization
    std::shared_ptr< ArArgumentBuilder > _args;
    std::shared_ptr< ArArgumentParser > _arg_parser;
};



RobotHandle::RobotHandle() :
    _robot(new ArRobot()),
    _args(new ArArgumentBuilder()),
    _arg_parser(new ArArgumentParser(_args)) {
        /* ... */
}


void init(const std::string& serial_port, size_t serial_baud, bool debug, const std::string& extra_args) {

    // adds any arguments given in /etc/Aria.args.
    // @note    useful on robots with unusual serial port or baud rate (e.g. pioneer lx)
    _arg_parser->loadDefaultArguments();

    // build argument list from ROS2 parameter server
    // @note    if serial port parameter contains a ':' character, then interpret it as hostname:tcpport
    //          for wireless serial connection. Otherwise, interpret it as only a serial port name/path.
    // @todo    move to ROSAria2Node::Parameters
    size_t colon_pos = config->serial_port.get().find(":");
    if (colon_pos != std::string::npos) {
        _args->add("-remoteHost"); // pass robot's hostname/IP address to Aria
        _args->add(config->serial_port.get().substr(0, colon_pos).c_str());
        _args->add("-remoteRobotTcpPort"); // pass robot's TCP port to Aria
        _args->add(config->serial_port.get().substr(colon_pos + 1).c_str());
    } else {
        _args->add("-robotPort %s", config->serial_port.get().c_str());
    }

    // if a baud rate was specified in baud parameter
    if (config->serial_baud != 0) {
        _args->add("-robotBaud %d", config->serial_baud);
    }

    // turn on all ARIA debugging
    if (config->debug_aria) {
        _args->add("-robotLogPacketsReceived"); // log received packets
        _args->add("-robotLogPacketsSent"); // log sent packets
        _args->add("-robotLogVelocitiesReceived"); // log received velocities
        _args->add("-robotLogMovementSent");
        _args->add("-robotLogMovementReceived");
        ArLog::init(ArLog::File, ArLog::Verbose, config->aria_log_filename.get().c_str(), true);
    }

    // add custom arguments passed in given *args*
    _args->add(extra_args.c_str());

    // connect to robot
    // @todo check if can be constructed before arguments are parsed
    _rc = std::make_shared< ArRobotConnector >(new ArRobotConnector(_arg_parser, _robot));
    if (!_rc->connectRobot()) {
        // RCLCPP_ERROR(this->get_logger(), "Aria could not connect to robot! (Check ~port parameter is correct, and permissions on port device, or any errors reported above)");
        throw std::runtime_error("Aria could not connect to robot! (Check ~port parameter is correct, and permissions on port device, or any errors reported above)");
    }

}



//------------------------------------------------------------------------------
/// @brief      Aria wrapper class implementing robot communication/control.
///             Restricts calls to Aria/AriaCoda to class definition, useful in modular design of ROS applications.
///
/// @note       Implemented as a singleton type, unsure if multiple instances of ArRobot can be run simultaneously on different serial devices.
///             (unclear after inspecting *rosaria* code)
///
class GlobalRobotHandle {
 public:
    //--------------------------------------------------------------------------
    /// @brief      Pointer type alias
    ///
    using Ptr = std::shared_ptr< GlobalRobotHandle >;

    //--------------------------------------------------------------------------
    /// @brief      Class instance (as pointer type)
    ///
    static Ptr instance;

    //--------------------------------------------------------------------------
    /// @brief      Static named constructor.
    ///
    /// @return     Pointer to new instance created, or to existing instance.
    ///
    /// @todo       Call init()?
    ///
    static Ptr get() {
        if (!instance) {
            // if no instance has been created, create one
            instance = Ptr(new RobotHandle());
        }
        return instance;
    }

    //--------------------------------------------------------------------------
    /// @brief      Destroys the object.
    ///
    virtual ~GlobalRobotHandle() = default;

    //--------------------------------------------------------------------------
    /// @brief      *no copy constructor declared*
    ///
    /// @note       Singleton types can't be copied, thus default copy constructor must be deleted.
    ///
    GlobalRobotHandle(const RobotHandle&) = delete;

    //--------------------------------------------------------------------------
    /// @brief      Initializes the object.
    ///
    /// @return     { description_of_the_return_value }
    ///
    int init(bool laser);

    const std::shared_ptr< ArRobot > robot;
    const std::shared_ptr< ArRobotConnector > connector;
    const std::shared_ptr< ArLaserConnector > laser_connector;

    //--------------------------------------------------------------------------
    /// @brief      Checks if robot handle has been initialized.
    ///
    /// @return     True if init() has been called, false otherwise.
    ///
    bool valid() const;

    //--------------------------------------------------------------------------
    /// @brief      Checks if initialized.
    ///
    /// @note       Wraps around valid() member, provided for convenience.
    ///
    operator bool() const;

protected:
    //--------------------------------------------------------------------------
    /// @brief      Constructs a new instance.
    ///
    /// @note       Singleton types can only be instantiated once, thus default constructor is private/protected.
    ///
    GlobalRobotHandle() = default;
};


GlobalRobotHandle::GlobalRobotHandle() :
    robot(new ArRobot()) {
        // only if embedded laser connection is required

}


int GlobalRobotHandle::init(bool laser) {

    robot = std::make_shared< ArRobot >(new ArRobot());

    // only needed within init()
    ArArgumentBuilder *args = new ArArgumentBuilder(); //  never freed
    ArArgumentParser *argparser = new ArArgumentParser(args); // Warning never freed
    argparser->loadDefaultArguments(); // adds any arguments given in /etc/Aria.args.  Useful on robots with unusual serial port or baud rate (e.g. pioneer lx)


    // connect to the robot
    connector = std::make_shared< ArRobotConnector >(new ArRobotConnector(argparser, robot));  // warning: should never be freed
    if (!conn->connectRobot()) {
        RCLCPP_ERROR(this->get_logger(), "Aria could not connect to robot! (Check ~port parameter is correct, and permissions on port device, or any errors reported above)");
        return 1;
    }


    // create laser connection (when configured)
    if(laser) {
        laser_connection = new ArLaserConnector(argparser, robot, conn);
    }

    // causes ARIA to load various robot-specific hardware parameters from the robot parameter file in /usr/local/Aria/params
    if (!Aria::parseArgs()) {
        RCLCPP_ERROR(this->get_logger(), "Aria error parsing startup parameters!");
        return 1;
    }

    return 0;
}


bool GlobalRobotHandle::valid() const {
    return robot;  // cast of pointer to bool is implicit
}


GlobalRobotHandle::operator bool() const {
    return this->valid();
}

}  // namespace aria

#endif  // ROSARIA2_INCLUDE_ROSARIA2_ARIAUTILS_HPP_
