#!/usr/bin/env python
import sys
import rospy

import smbus
from sensor_msgs.msg import Imu
from rtklm_i2c.BNO055 import BNO055

if __name__ == '__main__':
    rospy.init_node('rtklm_i2c_node')
    rospy.loginfo("rtklm_i2c_node started!")
    bus = smbus.SMBus(rospy.get_param("~bus"))

    imuPub = rospy.Publisher('/imu/data', Imu, queue_size=10)
    imuMsg = Imu()
    imu = BNO055(bus)
    
    imu_data_seq_counter = 0
    #rate = rospy.Rate(rospy.get_param("~frequency"))
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():

        imu.readCalib()
        if imu.status['sys'] > 0:
            rospy.loginfo("Calibration status sys:  " + str(imu.status['sys']))
        else:
            rospy.logwarn("Calibration status sys:  " + str(imu.status['sys']))
        if imu.status['gyro'] > 0:
            rospy.loginfo("Calibration status gyro:  " + str(imu.status['gyro']))
        else:
            rospy.logwarn("Calibration status gyro:  " + str(imu.status['gyro']))
        if imu.status['acc'] > 0:
            rospy.loginfo("Calibration status acc:  " + str(imu.status['acc']))
        else:
            rospy.logwarn("Calibration status acc:  " + str(imu.status['acc']))
        if imu.status['mag'] > 0:
            rospy.loginfo("Calibration status mag:  " + str(imu.status['mag']))
        else:
            rospy.logwarn("Calibration status mag:  " + str(imu.status['mag']))

        #imu.selfTest()
        #rospy.loginfo("Self test result mcu:  " + str(imu.result['mcu']))
        #rospy.loginfo("Self test result gyro: " + str(imu.result['gyro']))
        #rospy.loginfo("Self test result acc:  " + str(imu.result['acc']))
        #rospy.loginfo("Self test result mag:  " + str(imu.result['mag']))
        
        imuMsg.header.stamp = rospy.get_rostime()
        imuMsg.header.frame_id = rospy.get_param("~imuFrameId")
        imuMsg.header.seq = imu_data_seq_counter
        imu_data_seq_counter += 1

        imu.readGyro()
        imuMsg.angular_velocity.x = imu.gyro['x']
        imuMsg.angular_velocity.y = imu.gyro['y']
        imuMsg.angular_velocity.z = imu.gyro['z']
        imuMsg.angular_velocity_covariance[0] = 0.1
        imuMsg.angular_velocity_covariance[4] = 0.1
        imuMsg.angular_velocity_covariance[8] = 0.1

        imu.readLinAccel()
        imuMsg.linear_acceleration.x = imu.linAccel['x']
        imuMsg.linear_acceleration.y = imu.linAccel['y']
        imuMsg.linear_acceleration.z = imu.linAccel['z']
        imuMsg.linear_acceleration_covariance[0] = 1.0
        imuMsg.linear_acceleration_covariance[4] = 1.0
        imuMsg.linear_acceleration_covariance[8] = 1.0

        imu.readQuat()
        imuMsg.orientation.x = imu.quat['qx']
        imuMsg.orientation.y = imu.quat['qy']
        imuMsg.orientation.z = imu.quat['qz']
        imuMsg.orientation.w = imu.quat['qw']
        imuMsg.orientation_covariance[0] = 0.03
        imuMsg.orientation_covariance[4] = 0.03
        imuMsg.orientation_covariance[8] = 0.03

        imuPub.publish(imuMsg)
        rate.sleep()

    rospy.loginfo("finsihed!")