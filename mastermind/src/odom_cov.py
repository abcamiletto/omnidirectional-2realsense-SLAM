#!/usr/bin/env python
# coding: utf-8
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
from mastermind.cfg import odom_covConfig


rospy.init_node('saver', anonymous=True)

odom = Odometry()


# dynamic reconfigure
odom_gain_x = 1
odom_gain_y = 1
odom_gain_z = 1
def callback(config, level):
    global odom_gain
    odom_gain_x = config['odom_gain_post_x']
    odom_gain_y = config['odom_gain_post_y']
    odom_gain_z = config['odom_gain_post_z']
    print('Odom Gain set to: ' + str(odom_gain))
    return config
srv = Server(odom_covConfig, callback)

def Callback1(data):
    global odom, pub
    odom.header = data.header
    odom.pose.pose = data.pose.pose
    odom.twist.twist = data.twist.twist

    odom.twist.covariance[0] = data.twist.covariance[0] * odom_gain_x
    odom.twist.covariance[7] = data.twist.covariance[7] * odom_gain_y
    odom.twist.covariance[35] = data.twist.covariance[35]  * odom_gain_z
        
    pub.publish(odom)
    

pub = rospy.Publisher('/robot_odom_wheels_2', Odometry, queue_size=2)
rospy.Subscriber("/robot_odom_wheels", Odometry, Callback1, queue_size = 1)



rospy.spin()

