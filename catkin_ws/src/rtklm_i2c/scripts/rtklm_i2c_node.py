#!/usr/bin/env python
import sys
import rospy

from smbus import SMBus
from imusensor.MPU9250 import MPU9250
from imusensor.filters import kalman
from sensor_msgs.msg import Imu
import tf.transformations 

if __name__ == '__main__':
    rospy.init_node('rtklm_i2c_node')
    rospy.loginfo("rtklm_i2c_node started!")
    pub = rospy.Publisher('/imu', Imu, queue_size=10)
    msg = Imu()

    imu = MPU9250.MPU9250(SMBus(1), 0x68)
    imu.begin()
    imu.readSensor()
    imu.computeOrientation()
    
    sensorfusion = kalman.Kalman()
    sensorfusion.roll = imu.roll
    sensorfusion.pitch = imu.pitch
    sensorfusion.yaw = imu.yaw
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        imu.readSensor()
        
        dt = 1
        
        sensorfusion.computeAndUpdateRollPitchYaw(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2], imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2], imu.MagVals[0], imu.MagVals[1], imu.MagVals[2], dt)

        msg.header.stamp = rospy.get_rostime()
        quaternion = tf.transformations.quaternion_from_euler(sensorfusion.roll, sensorfusion.pitch, sensorfusion.yaw)
        msg.angular_velocity.x = imu.GyroVals[0]
        msg.angular_velocity.y = imu.GyroVals[1]
        msg.angular_velocity.z = imu.GyroVals[2]
        msg.linear_acceleration.x = imu.AccelVals[0]
        msg.linear_acceleration.y = imu.AccelVals[1]
        msg.linear_acceleration.z = imu.AccelVals[2]
        msg.orientation.x = quaternion[0]
        msg.orientation.y = quaternion[1]
        msg.orientation.z = quaternion[2]
        msg.orientation.w = quaternion[3]

        #rospy.loginfo("Accel x: {0} ; Accel y : {1} ; Accel z : {2}".format(imu.AccelVals[0], imu.AccelVals[1], imu.AccelVals[2]))
        #rospy.loginfo("Gyro x: {0} ; Gyro y : {1} ; Gyro z : {2}".format(imu.GyroVals[0], imu.GyroVals[1], imu.GyroVals[2]))
        #rospy.loginfo("Mag x: {0} ; Mag y : {1} ; Mag z : {2}".format(imu.MagVals[0], imu.MagVals[1], imu.MagVals[2]))
        #rospy.loginfo(("roll: {0} ; pitch : {1} ; yaw : {2}".format(imu.roll, imu.pitch, imu.yaw)))

        pub.publish(msg)
        rate.sleep()

    rospy.loginfo("finsihed!")
