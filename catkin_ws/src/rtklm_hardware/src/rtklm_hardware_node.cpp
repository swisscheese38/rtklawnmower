#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
using namespace std;

class RtklmInterface : public hardware_interface::RobotHW
{
public:
  RtklmInterface(string port, unsigned long baud) {
     
    pos_[0] = 0.0; pos_[1] = 0.0;
    vel_[0] = 0.0; vel_[1] = 0.0;
    eff_[0] = 0.0; eff_[1] = 0.0;
    cmd_[0] = 0.0; cmd_[1] = 0.0;

    hardware_interface::JointStateHandle state_handle_1("left_wheel_joint", &pos_[0], &vel_[0], &eff_[0]);
    jnt_state_interface_.registerHandle(state_handle_1);

    hardware_interface::JointStateHandle state_handle_2("right_wheel_joint", &pos_[1], &vel_[1], &eff_[1]);
    jnt_state_interface_.registerHandle(state_handle_2);

    registerInterface(&jnt_state_interface_);

    hardware_interface::JointHandle vel_handle_1(jnt_state_interface_.getHandle("left_wheel_joint"), &cmd_[0]);
    jnt_vel_interface_.registerHandle(vel_handle_1);

    hardware_interface::JointHandle vel_handle_2(jnt_state_interface_.getHandle("right_wheel_joint"), &cmd_[1]);
    jnt_vel_interface_.registerHandle(vel_handle_2);

    registerInterface(&jnt_vel_interface_);
  }

  ~RtklmInterface(){
    //
  }

  ros::Time getTime() const {return ros::Time::now();}
  ros::Duration getPeriod() const {return ros::Duration(0.01);}

  void read(){
    ROS_INFO_STREAM("Commands for joints: " << cmd_[0] << ", " << cmd_[1]);
    //send commands to motors
    //K1.s((int32_t)  cmd_[0] * radiansToTicks);
    //K2.s((int32_t) -cmd_[1] * radiansToTicks);
  }

  void write(){
    //pos_[0] =  resultGetK1P.value() * ticksToRadians;
    //pos_[1] = -resultGetK2P.value() * ticksToRadians;
    //vel_[0] =  K1.getS().value() * ticksToRadians;
    //vel_[1] = -K2.getS().value() * ticksToRadians;
  }

private:
  hardware_interface::JointStateInterface    jnt_state_interface_;
  hardware_interface::VelocityJointInterface jnt_vel_interface_;
  double ticksToRadians;
  double radiansToTicks;
  double cmd_[2];
  double pos_[2];
  double vel_[2];
  double eff_[2];

};

int main(int argc, char **argv)
{
  double x, y, theta;
  
  ros::init(argc, argv, "rtklm_hardware_node");
  ros::NodeHandle nh;
 
 //TODO: read port and baud as parameters
  string port = "/dev/ttyUSB0";
  unsigned long baud = 9600;

  RtklmInterface robot(port, baud);
  ROS_INFO_STREAM("period: " << robot.getPeriod().toSec());
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Rate rate(1.0 / robot.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while(ros::ok())
  {
    //TODO: Detect loss of connection to robot and reconnect
    robot.read();
    robot.write();
    cm.update(robot.getTime(), robot.getPeriod());
    rate.sleep();
  }
  spinner.stop();

  return 0;
}