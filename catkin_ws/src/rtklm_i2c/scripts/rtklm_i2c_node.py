#!/usr/bin/env python
import sys
import rospy

import smbus
from sensor_msgs.msg import Imu
from rtklm_i2c.BNO055 import BNO055

if __name__ == '__main__':
    rospy.init_node('rtklm_i2c_node')
    rospy.loginfo("rtklm_i2c_node started!")
    bus = smbus.SMBus(1)

    imuPub = rospy.Publisher('/imu', Imu, queue_size=10)
    imuMsg = Imu()
    imu = BNO055(bus)
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        
        imuMsg.header.stamp = rospy.get_rostime()
        imuMsg.header.frame_id = "imu_frame"

        imu.readGyro()
        imuMsg.angular_velocity.x = imu.gyro['x']
        imuMsg.angular_velocity.y = imu.gyro['y']
        imuMsg.angular_velocity.z = imu.gyro['z']

        imu.readLinAccel()
        imuMsg.linear_acceleration.x = imu.linAccel['x']
        imuMsg.linear_acceleration.y = imu.linAccel['y']
        imuMsg.linear_acceleration.z = imu.linAccel['z']

        imu.readQuat()
        imuMsg.orientation.x = imu.quat['qx']
        imuMsg.orientation.y = imu.quat['qy']
        imuMsg.orientation.z = imu.quat['qz']
        imuMsg.orientation.w = imu.quat['qw']

        imuPub.publish(imuMsg)
        rate.sleep()

    rospy.loginfo("finsihed!")