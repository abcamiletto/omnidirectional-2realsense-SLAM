#!/usr/bin/env python
# coding: utf-8
import rospy
import numpy as np
from nav_msgs.msg import Odometry

history = np.zeros([3, 10*100])
counter = 0

def Callback(data):
    global history, counter
    history[0, counter] = data.twist.twist.linear.x
    history[1, counter] = data.twist.twist.linear.y
    history[2, counter] = data.twist.twist.angular.z
    if counter < 10*100 - 1:
        counter += 1
    else:
        history = np.roll(history, -1, axis=1)

rospy.init_node('saver', anonymous=True)
rospy.Subscriber("robot_odom_wheels", Odometry, Callback)

r = rospy.Rate(3)
while not rospy.is_shutdown():
    cov_matx = np.cov(history)
    print(cov_matx)
    print(' ')
    r.sleep()
