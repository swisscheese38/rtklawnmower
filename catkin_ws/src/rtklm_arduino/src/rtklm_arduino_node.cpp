#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <serial/serial.h>
#include <math.h>
#include <string.h>

class RtklmArduinoInterface : public hardware_interface::RobotHW {

public:
  RtklmArduinoInterface(std::string port, unsigned long baud) {

    try {
        arduino.setPort(port);
        arduino.setBaudrate(baud);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        arduino.setTimeout(to);
        arduino.open();
    } catch(serial::IOException &err) {
        ROS_ERROR_STREAM("Connection failed with port: " << arduino.getPort());
        ros::shutdown();
    }
    ROS_INFO("Connection established with Robot");

    std::string left_joint_name, right_joint_name;
    ros::param::get("~left_wheel", left_joint_name);
    ros::param::get("~right_wheel", right_joint_name);
    double ticksPerRevolution;
    ros::param::get("~ticksPerRevolution", ticksPerRevolution);
    ticksToRadians = (2.0*M_PI)/ticksPerRevolution;
    radiansToTicks = 1.0/ticksToRadians;

    hardware_interface::JointStateHandle state_handle_1(left_joint_name, &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_1);

    hardware_interface::JointStateHandle state_handle_2(right_joint_name, &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_2);

    registerInterface(&jnt_state_interface);

    hardware_interface::JointHandle vel_handle_1(jnt_state_interface.getHandle(left_joint_name), &cmd[0]);
    jnt_vel_interface.registerHandle(vel_handle_1);

    hardware_interface::JointHandle vel_handle_2(jnt_state_interface.getHandle(right_joint_name), &cmd[1]);
    jnt_vel_interface.registerHandle(vel_handle_2);

    registerInterface(&jnt_vel_interface);
  }

  void read(const ros::Duration &period) {
    //read speeds from motors
    while (arduino.available() > 0) {
      std::string line = arduino.readline();
      ROS_INFO_STREAM("Read from Arduino: " << line);
      std::stringstream lineBuffer(line);
      std::string lvel, rvel;
      std::getline(lineBuffer, lvel, ' ');
      std::getline(lineBuffer, rvel, ' ');
      try {
        vel[0] = -(ticksToRadians * std::stod(lvel));
        vel[1] = +(ticksToRadians * std::stod(rvel));
      } catch(std::invalid_argument&) {
        ROS_WARN_STREAM("Bad line received: " << line);
      }
    }
    pos[0] += (vel[0] * period.toSec());
    pos[1] += (vel[1] * period.toSec());
  }

  void write() {
    //send commands to motors
    std::ostringstream out;
    out.precision(0);
    out << std::fixed << -(cmd[0] * radiansToTicks);
    out << " ";
    out << std::fixed << +(cmd[1] * radiansToTicks);
    out << "\n";
    std::string s = out.str();
    ROS_INFO_STREAM("Command for joints: " << s);
    arduino.write(s);
  }

private:
  serial::Serial arduino;
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  double cmd[2] = {0,0};
  double pos[2] = {0,0};
  double vel[2] = {0,0};
  double eff[2] = {0,0};
  double ticksToRadians;
  double radiansToTicks;
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "rtklm_arduino_node");
  ros::NodeHandle nh;
 
  std::string port;
  int baud;
  int frequency;
  ros::param::get("~usbPort", port);
  ros::param::get("~usbBaudrate", baud);
  ros::param::get("~frequency", frequency);

  RtklmArduinoInterface robot(port, baud);
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Rate rate(frequency);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while(ros::ok())
  {
    robot.read(rate.expectedCycleTime());
    robot.write();
    cm.update(ros::Time::now(), rate.expectedCycleTime());
    rate.sleep();
  }
  spinner.stop();
  return 0;
}
