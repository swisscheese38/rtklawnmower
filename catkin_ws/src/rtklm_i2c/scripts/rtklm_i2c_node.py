#!/usr/bin/env python
import sys
import rospy

import smbus
from sensor_msgs.msg import Imu
from rtklm_i2c.BNO055 import BNO055

def calculateNextCalibrationCheck():
    return rospy.Time.now() + rospy.rostime.Duration(60)

if __name__ == '__main__':
    rospy.init_node('rtklm_i2c_node')
    rospy.loginfo("rtklm_i2c_node started!")
    bus = smbus.SMBus(rospy.get_param("~bus"))

    imuPub = rospy.Publisher('/imu/data', Imu, queue_size=10)
    imuMsg = Imu()
    calibrationBytes = rospy.get_param("~calibration")
    if calibrationBytes == None or len(calibrationBytes) != 22:
        rospy.logwarn("No initial IMU calibration provided")
        imu = BNO055(bus)
    else:
        imu = BNO055(bus, 0x28, calibrationBytes)

    rate = rospy.Rate(rospy.get_param("~frequency"))
    nextCalibrationCheck = rospy.Time.now()

    while not rospy.is_shutdown():

        # occasionally check calibration status
        if nextCalibrationCheck < rospy.Time.now():
            imu.readCalib()
            if not all(v == 3 for v in imu.status.values()):
                rospy.logwarn("IMU not fully calibrated: " + str(imu.status))
            imu.selfTest()
            if not all(v == 1 for v in imu.result.values()):
                rospy.logwarn("IMU didn't pass selftest: " + str(imu.result))
            nextCalibrationCheck = calculateNextCalibrationCheck()
        
        imuMsg.header.stamp = rospy.get_rostime()
        imuMsg.header.frame_id = rospy.get_param("~imuFrameId")

        imu.readQuat()
        imuMsg.orientation.x = imu.quat['qx']
        imuMsg.orientation.y = imu.quat['qy']
        imuMsg.orientation.z = imu.quat['qz']
        imuMsg.orientation.w = imu.quat['qw']
        imuMsg.orientation_covariance[0] = 0.03 # covariance in x,y,z (3x3)
        imuMsg.orientation_covariance[4] = 0.03
        imuMsg.orientation_covariance[8] = 0.03

        imu.readLinAccel()
        imuMsg.linear_acceleration.x = imu.linAccel['x']
        imuMsg.linear_acceleration.y = imu.linAccel['y']
        imuMsg.linear_acceleration.z = imu.linAccel['z']
        imuMsg.linear_acceleration_covariance[0] = 1.00 # covariance in x,y,z (3x3)
        imuMsg.linear_acceleration_covariance[4] = 1.00
        imuMsg.linear_acceleration_covariance[8] = 1.00

        imu.readGyro()
        imuMsg.angular_velocity.x = round(imu.gyro['x'], 5)
        imuMsg.angular_velocity.y = round(imu.gyro['y'], 5)
        imuMsg.angular_velocity.z = round(imu.gyro['z'], 5)
        imuMsg.angular_velocity_covariance[0] = 0.10 # covariance in x,y,z (3x3)
        imuMsg.angular_velocity_covariance[4] = 0.10
        imuMsg.angular_velocity_covariance[8] = 0.10

        imuPub.publish(imuMsg)
        rate.sleep()

    rospy.loginfo("finsihed!")