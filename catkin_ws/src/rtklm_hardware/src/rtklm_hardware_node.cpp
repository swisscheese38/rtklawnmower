#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <serial/serial.h>

class RtklmInterface : public hardware_interface::RobotHW {

public:
  RtklmInterface(std::string port, unsigned long baud) {

    serial.setPort(port);
    try {
        serial.open();
    } catch(serial::IOException &err) {
        ROS_ERROR_STREAM("Connection failed with port: " << serial.getPort());
        ros::shutdown();
    }
    ROS_INFO("Connection established with Robot");
    serial.setBaudrate(baud);
    serial::Timeout to = serial::Timeout::simpleTimeout(500);
    serial.setTimeout(to);

    hardware_interface::JointStateHandle state_handle_1("left_wheel_joint", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_1);

    hardware_interface::JointStateHandle state_handle_2("right_wheel_joint", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_2);

    registerInterface(&jnt_state_interface);

    hardware_interface::JointHandle vel_handle_1(jnt_state_interface.getHandle("left_wheel_joint"), &cmd[0]);
    jnt_vel_interface.registerHandle(vel_handle_1);

    hardware_interface::JointHandle vel_handle_2(jnt_state_interface.getHandle("right_wheel_joint"), &cmd[1]);
    jnt_vel_interface.registerHandle(vel_handle_2);

    registerInterface(&jnt_vel_interface);
  }

  void read(){
    ROS_INFO_STREAM("States for joints: " << pos[0] << ", " << pos[1] << ", " << vel[0] << ", " << vel[1]);
    std::string result;
    result = serial.read(serial.available());
    ROS_INFO_STREAM("Read: " << result);
    //pos_[0] =  resultGetK1P.value() * ticksToRadians;
    //pos_[1] = -resultGetK2P.value() * ticksToRadians;
    //vel_[0] =  K1.getS().value() * ticksToRadians;
    //vel_[1] = -K2.getS().value() * ticksToRadians;

  }

  void write(){
    ROS_INFO_STREAM("Commands for joints: " << cmd[0] << ", " << cmd[1]);
    //send commands to motors
    //serial.write(cmd[0] << " " << cmd[1] << "\n")
  }

private:
  serial::Serial serial;
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  double cmd[2] = {0,0};
  double pos[2] = {0,0};
  double vel[2] = {0,0};
  double eff[2] = {0,0};

};

int main(int argc, char **argv)
{
  double x, y, theta;
  
  ros::init(argc, argv, "rtklm_hardware_node");
  ros::NodeHandle nh;
 
  std::string port = "/dev/ttyUSB0";
  unsigned long baud = 9600;
  RtklmInterface robot(port, baud);
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Rate rate(10);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while(ros::ok())
  {
    robot.read();
    robot.write();
    cm.update(ros::Time::now(), rate.expectedCycleTime());
    rate.sleep();
  }
  spinner.stop();
  return 0;
}