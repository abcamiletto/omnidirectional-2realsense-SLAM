#!/usr/bin/env python
# coding: utf-8
import rospy
import numpy as np
from sensor_msgs.msg import Imu

history = np.zeros([6, 10*100])
counter = 0

def Callback(data):
    global history, counter
    history[0, counter] = data.angular_velocity.x
    history[1, counter] = data.angular_velocity.y
    history[2, counter] = data.angular_velocity.z
    history[3, counter] = data.linear_acceleration.x
    history[4, counter] = data.linear_acceleration.y
    history[5, counter] = data.linear_acceleration.z
    if counter < 10*100 - 1:
        counter += 1
    else:
        np.roll(history, 1, axis=1)
        

rospy.init_node('saver', anonymous=True)
rospy.Subscriber("camera1/imu", Imu, Callback)

r = rospy.Rate(1)
while not rospy.is_shutdown():
    cov_matx = np.cov(history)
    print(cov_matx)
    r.sleep()
