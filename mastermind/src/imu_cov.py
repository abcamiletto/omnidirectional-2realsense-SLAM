#!/usr/bin/env python
# coding: utf-8
import rospy
import numpy as np
from sensor_msgs.msg import Imu
from dynamic_reconfigure.server import Server
from mastermind.cfg import imu_covarianceConfig


rospy.init_node('saver', anonymous=True)

imu1 = Imu()
imu2 = Imu()

# dynamic reconfigure
gain_imu = 1
def callback(config, level):
    global gain_imu
    gain_imu = config['imu_gain']
    print('IMU Gain set to: ' + str(gain_imu))
    return config
srv = Server(imu_covarianceConfig, callback)

def Callback1(data):
    global imu1, pub1
    imu1.angular_velocity = data.angular_velocity
    imu1.linear_acceleration = data.linear_acceleration
    
    if abs(data.angular_velocity.y) < 1e-2:
        imu1.angular_velocity_covariance[0] = 1e-4 * gain_imu
        imu1.angular_velocity_covariance[4] = 8e-5 * gain_imu
        imu1.angular_velocity_covariance[8] = 5e-5 * gain_imu
        imu1.linear_acceleration_covariance[0] = 7e-2 * gain_imu
        imu1.linear_acceleration_covariance[4] = 1.4e-4 * gain_imu
        imu1.linear_acceleration_covariance[8] = 3e-1 * gain_imu
    else:
        imu1.angular_velocity_covariance[0] = 1e-4 * gain_imu
        imu1.angular_velocity_covariance[4] = 8e-2 * gain_imu
        imu1.angular_velocity_covariance[8] = 1e-4 * gain_imu
        imu1.linear_acceleration_covariance[0] = 3.1e-1 * gain_imu
        imu1.linear_acceleration_covariance[4] = 2e-1 * gain_imu
        imu1.linear_acceleration_covariance[8] = 5e-1 * gain_imu
        
    pub1.publish(imu1)
    
def Callback2(data):
    global imu2, pub2
    imu2.angular_velocity = data.angular_velocity
    imu2.linear_acceleration = data.linear_acceleration
    
    if abs(data.angular_velocity.y) < 1e-2:
        imu2.angular_velocity_covariance[0] = 1e-4 * gain_imu
        imu2.angular_velocity_covariance[4] = 8e-5 * gain_imu
        imu2.angular_velocity_covariance[8] = 5e-5 * gain_imu
        imu2.linear_acceleration_covariance[0] = 7e-2 * gain_imu
        imu2.linear_acceleration_covariance[4] = 1.4e-4 * gain_imu
        imu2.linear_acceleration_covariance[8] = 3e-1 * gain_imu
    else:
        imu2.angular_velocity_covariance[0] = 1e-4 * gain_imu
        imu2.angular_velocity_covariance[4] = 8e-2 * gain_imu
        imu2.angular_velocity_covariance[8] = 1e-4 * gain_imu
        imu2.linear_acceleration_covariance[0] = 3.1e-1 * gain_imu
        imu2.linear_acceleration_covariance[4] = 2e-1 * gain_imu
        imu2.linear_acceleration_covariance[8] = 5e-1 * gain_imu
        
    pub2.publish(imu2)
    

pub1 = rospy.Publisher('/camera1/imu_filtered', Imu, queue_size=10)
pub2 = rospy.Publisher('/camera2/imu_filtered', Imu, queue_size=10)
rospy.Subscriber("/camera1/imu", Imu, Callback1)
rospy.Subscriber("/camera2/imu", Imu, Callback2)


rospy.spin()

