#include <ros/ros.h>
#include <iostream>
#include <sstream>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.hpp>
#include <string>
#include "rtklm_hw.hpp"
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <math.h>
#include <serial/serial.h>

#define PORT "/dev/ttyUSB1"
#define BAUD_RATE 9600
#define TIMEOUT 500

namespace rtklm_hardware_interface
{

    serial::Serial arduino;
    hardware_interface::JointStateInterface state_interface;
    hardware_interface::VelocityJointInterface vel_interface;
    ros::NodeHandle nh;
    double cmd[2] = {0,0};
    double pos[2] = {0,0};
    double vel[2] = {0,0};
    double eff[2] = {0,0};

    RtklmInterface::RtklmInterface()
    {
    }

    RtklmInterface::~RtklmInterface()
    {
        arduino.close();
    }

    bool RtklmInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
    {
        try {
            arduino.setPort(PORT);
            arduino.setBaudrate(BAUD_RATE);
            serial::Timeout to = serial::Timeout::simpleTimeout(TIMEOUT);
            arduino.setTimeout(to);
            arduino.open();
        } catch (serial::IOException& e) {
            ROS_ERROR_STREAM("Unable to open serial port");
            ros::shutdown();
            return false;
        }

        if (arduino.isOpen()) {
            ROS_INFO_STREAM("Serial Port initialized");
        } else {
            ros::shutdown();
            return false;
        }

        hardware_interface::JointStateHandle left_joint("left_wheel_joint", &pos[0], &vel[0], &eff[0]);
        state_interface.registerHandle(left_joint);

        hardware_interface::JointStateHandle right_joint("right_wheel_joint", &pos[1], &vel[1], &eff[1]);
        state_interface.registerHandle(right_joint);

        registerInterface(&state_interface);

        hardware_interface::JointHandle left_vel(state_interface.getHandle("left_wheel"), &cmd[0]);
        vel_interface.registerHandle(left_vel);

        hardware_interface::JointHandle right_vel(state_interface.getHandle("right_wheel"), &cmd[1]);
        vel_interface.registerHandle(right_vel);

        registerInterface(&vel_interface);

        return true;
    }

    void RtklmInterface::read(const ros::Time &time, const ros::Duration &period)
    {
        if (arduino.available()) {
            ROS_INFO_STREAM("Reading from serial port");
            std::string result = arduino.read(arduino.available());
            ROS_INFO_STREAM("Read: " << result);

            std::string lpos, rpos, lvel, rvel;
            std::stringstream sstream;

            sstream.str(result);
            std::getline(sstream, lpos, ' ');
            std::getline(sstream, lvel, ' ');
            std::getline(sstream, rpos, ' ');
            std::getline(sstream, rvel, ' ');
            pos[0] = stod(lpos);
            vel[0] = stod(lvel);
            pos[1] = stod(rpos);
            vel[1] = stod(rvel);
        }
    }

    void RtklmInterface::write(const ros::Time &time, const ros::Duration &period)
    {
        std::string command = std::to_string(cmd[0]) + " " + std::to_string(cmd[1]);
        ROS_INFO_STREAM("Writing to serial port" << command);
        arduino.write(command);
    }

} // namespace rtklm_hardware_interface
PLUGINLIB_EXPORT_CLASS(rtklm_hardware_interface::RtklmInterface, hardware_interface::RobotHW)