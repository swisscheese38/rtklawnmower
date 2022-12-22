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
#define PULSES_PER_REV 1496.0
#define WHEEL_DIAMETER 0.065

namespace rtklm_hardware_interface
{

    RtklmInterface::RtklmInterface()
    {
    }

    RtklmInterface::~RtklmInterface()
    {
    }

    bool RtklmInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
    {
        //TODO
        return true;
    }

    void RtklmInterface::read(const ros::Time &time, const ros::Duration &period)
    {
        //TODO
    }

    void RtklmInterface::write(const ros::Time &time, const ros::Duration &period)
    {
        //TODO
    }

} // namespace rtklm_hardware_interface
PLUGINLIB_EXPORT_CLASS(rtklm_hardware_interface::RtklmInterface, hardware_interface::RobotHW)