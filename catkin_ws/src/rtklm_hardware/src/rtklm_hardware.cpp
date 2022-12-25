#include <ros/ros.h>
#include <iostream>
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
#include <sstream>
#include <math.h>
#include <serial/serial.h>
#define PULSES_PER_REV 1496.0
#define WHEEL_DIAMETER 0.065

namespace rtklm_hardware_interface
{

    serial::Serial ser;

    RtklmInterface::RtklmInterface()
    {
    }

    RtklmInterface::~RtklmInterface()
    {
    }

    bool RtklmInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
    {
        try {
            ser.setPort("/dev/ttyUSB1");
            ser.setBaudrate(9600);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open();
        } catch (serial::IOException& e) {
            ROS_ERROR_STREAM("Unable to open serial port");
            return false;
        }

        if (ser.isOpen()) {
            ROS_INFO_STREAM("Serial Port initialized");
        } else {
            return false;
        }

        return true;
    }

    void RtklmInterface::read(const ros::Time &time, const ros::Duration &period)
    {
        if (ser.available()) {
            ROS_INFO_STREAM("Reading from serial port");
            std::string result = ser.read(ser.available());
            ROS_INFO_STREAM("Read: " << result);
        }
    }

    void RtklmInterface::write(const ros::Time &time, const ros::Duration &period)
    {
        std::string command = "0.0 0.0";
        ROS_INFO_STREAM("Writing to serial port" << command);
        ser.write(command);
    }

} // namespace rtklm_hardware_interface
PLUGINLIB_EXPORT_CLASS(rtklm_hardware_interface::RtklmInterface, hardware_interface::RobotHW)