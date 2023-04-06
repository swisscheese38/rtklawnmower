#!/usr/bin/env python
import sys
import rospy

import smbus
from sensor_msgs.msg import Imu
from rtklm_i2c.BNO055 import BNO055

def calculateNextCalibrationCheck():
    return rospy.Time.now() + rospy.rostime.Duration(5)

if __name__ == '__main__':
    rospy.init_node('rtklm_i2c_node')
    rospy.loginfo("rtklm_i2c_node started!")
    bus = smbus.SMBus(rospy.get_param("~bus"))

    imuPub = rospy.Publisher('/imu/data', Imu, queue_size=10)
    imuMsg = Imu()
    imu = BNO055(bus)
    
    rate = rospy.Rate(rospy.get_param("~frequency"))
    nextCalibrationCheck = calculateNextCalibrationCheck()

    while not rospy.is_shutdown():

        # occasionally check calibration status
        if nextCalibrationCheck < rospy.Time.now():
            imu.readCalib()
            for calibstatus_key, calibstatus_val in imu.status.items():
                if calibstatus_val != 3:
                    rospy.logwarn("IMU " + calibstatus_key + " not fully calibrated: " + str(calibstatus_val))
            imu.selfTest()
            for selftest_key, selftest_val in imu.result.items():
                if selftest_val != 1:
                    rospy.logwarn("IMU " + selftest_key + " didn't pass selftest: " + str(selftest_val))
            nextCalibrationCheck = calculateNextCalibrationCheck()
        
        imuMsg.header.stamp = rospy.get_rostime()
        imuMsg.header.frame_id = rospy.get_param("~imuFrameId")

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