#include <ros/ros.h>
#include <iostream>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.hpp>
#include <string>

namespace rtklm_hardware_interface
{

    class RtklmInterface : public hardware_interface::RobotHW
    {
    public:
        RtklmInterface();
        ~RtklmInterface();
        bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh);
        void read(const ros::Time &time, const ros::Duration &period);
        void write(const ros::Time &time, const ros::Duration &period);

    protected:
        ros::NodeHandle nh_;

        //interfaces
        hardware_interface::JointStateInterface joint_state_interface;
        hardware_interface::EffortJointInterface effort_joint_interface;

        int num_joints;
        std::vector<std::string> joint_name;

        //temp states
        std::vector<double> temp_joint_position_state;
        std::vector<double> temp_joint_velocity_state;

        //actual states
        std::vector<double> joint_position_state;
        std::vector<double> joint_velocity_state;
        std::vector<double> joint_effort_state;

        //given setpoints
        std::vector<double> joint_effort_command;

        //MyRobot1CPP* robot;
    private:
        std::vector<float> message_vals;
    };
} // namespace rtklm_hardware_interface