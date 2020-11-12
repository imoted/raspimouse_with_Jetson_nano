#!/usr/bin/env python
# coding: utf-8

import MPU6050
import time
import sys
import rospy 
import math
from sensor_msgs.msg import Imu

class MPU6050_class:
    def __init__(self):
            rospy.init_node("MPU6050_node")
            self.mpu6050 = MPU6050.MPU6050()
            self.imu_pub = rospy.Publisher("imu", Imu, queue_size=10)
            self.imu_data = Imu()
            param_frame = rospy.get_param("param_imu_frame", default="0")
            self.imu_data.header.frame_id = param_frame
            self.read_loop()

    def read_loop(self):
        rate = rospy.Rate(100) 
        while not rospy.is_shutdown():
            self.imu_data.header.stamp = rospy.get_rostime()
            accel = self.mpu6050.readAccel()
            self.imu_data.linear_acceleration.x = accel['x'] * 9.80665
            self.imu_data.linear_acceleration.y = accel['y'] * 9.80665
            self.imu_data.linear_acceleration.z = accel['z'] * 9.80665
            gyro = self.mpu6050.readGyro()
            self.imu_data.angular_velocity.x = gyro['x'] * math.pi / 180.0
            self.imu_data.angular_velocity.y = gyro['y'] * math.pi / 180.0
            self.imu_data.angular_velocity.z = gyro['z'] * math.pi / 180.0          

            self.imu_pub.publish(self.imu_data)
            rate.sleep()

if __name__ == '__main__':
    mpu6050 = MPU6050_class()


