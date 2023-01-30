#!/usr/bin/env python
import sys
import rospy
import smbus

from imusensor.MPU9250 import MPU9250

if __name__ == '__main__':
    rospy.init_node('rtklm_i2c_node')
    rospy.loginfo("rtklm_i2c_node started!")
    rate = rospy.Rate(1)

    bus = smbus.SMBus(1)
    imu = MPU9250.MPU9250(bus, 0x68)
    imu.begin()

    while not rospy.is_shutdown():
        imu.readSensor()
        imu.computeOrientation()
        rospy.loginfo("Accel x: {0} ; Accel y : {1} ; Accel z : {2}".format(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2]))
        rospy.loginfo("Gyro x: {0} ; Gyro y : {1} ; Gyro z : {2}".format(imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2]))
        rospy.loginfo("Mag x: {0} ; Mag y : {1} ; Mag z : {2}".format(imu.MagVals[0], imu.MagVals[1], imu.MagVals[2]))
        rospy.loginfo(("roll: {0} ; pitch : {1} ; yaw : {2}".format(imu.roll, imu.pitch, imu.yaw)))
        rate.sleep()

    rospy.loginfo("finsihed!")
