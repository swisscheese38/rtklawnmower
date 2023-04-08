#!/usr/bin/env python
import sys
import rospy

import smbus
from rtklm_i2c.BNO055 import BNO055

if __name__ == '__main__':
    rospy.init_node('imu_calibration')
    rospy.loginfo("imu_calibration started!")
    rospy.loginfo("node will terminate when fully calibrated")
    rospy.loginfo("move IMU as per manufacturer's instructions until all sensor's report status 3")
    bus = smbus.SMBus(rospy.get_param("~bus"))

    imu = BNO055(bus)

    rate = rospy.Rate(1)
    fullyCalibrated = False

    while not rospy.is_shutdown() and not fullyCalibrated:
        imu.readCalib()
        rospy.loginfo("IMU calibration status: " + str(imu.status))
        fullyCalibrated = all(v == 3 for v in imu.status.values())
        rate.sleep()

    rospy.loginfo("imu_calibration finsihed!")
    byteString = ""
    for b in imu.readCalibrationBytes():
        byteString += hex(b) + ","

    #offsets_accelerometer  (6 bytes) BNO055_ACC_OFFSET_X_LSB = 0x55
    #offsets_magnetometer   (6 bytes) BNO055_MAG_OFFSET_X_LSB = 0x5B
    #offsets_gyroscope      (6 bytes) BNO055_GYR_OFFSET_X_LSB = 0x61
    #radius_accelerometer   (2 bytes) BNO055_ACC_RADIUS_LSB   = 0x67
    #radius_magnetometer    (2 bytes) BNO055_MAG_RADIUS_LSB   = 0x69
    
    rospy.loginfo("use the following 22 calibration bytes: " + str(byteString))
    rospy.loginfo("This corresponds to: " + str(imu.readCalibOffsets()))
    
