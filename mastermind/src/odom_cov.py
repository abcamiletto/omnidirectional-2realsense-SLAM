#!/usr/bin/env python
# coding: utf-8
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from dynamic_reconfigure.server import Server
from mastermind.cfg import odom_covConfig


rospy.init_node('saver', anonymous=True)

odom = Odometry()
odom2 = Odometry()


# dynamic reconfigure
odom_gain_xy = 1
odom_gain_z = 1
vo_gain = 1
vo_gain_z = 1
def callback(config, level):
    global odom_gain_xy, odom_gain_z, vo_gain, vo_gain_z
    odom_gain_xy = config['odom_gain_post_xy']
    odom_gain_z = config['odom_gain_post_z']
    vo_gain = config['vo_gain']
    vo_gain_z = config['vo_gain_z']
    print('Odom Gain xy set to: ' + str(odom_gain_xy))
    print('Odom Gain z set to: ' + str(odom_gain_z))
    print('VO Gain set to: ' + str(vo_gain))
    print('VO Gain z set to: ' + str(vo_gain_z))
    return config
srv = Server(odom_covConfig, callback)
counter = 0
pose_offset = Odometry()
def Callback1(data):
    global odom, pub, counter, pose_offset
    if counter < 10:
        pose_offset.pose.pose = data.pose.pose
        counter += 1
    odom.header = data.header
    odom.child_frame_id = data.child_frame_id
    odom.pose.pose = data.pose.pose
    odom.pose.pose.position.x = data.pose.pose.position.x - pose_offset.pose.pose.position.x
    odom.pose.pose.position.y = data.pose.pose.position.y - pose_offset.pose.pose.position.y   
    odom.twist.twist = data.twist.twist
    odom.twist.covariance[0] = data.twist.covariance[0] * odom_gain_xy
    odom.twist.covariance[7] = data.twist.covariance[7] * odom_gain_xy
    odom.twist.covariance[35] = data.twist.covariance[35]  * odom_gain_z
        
    pub.publish(odom)
    
def Callback2(data):
    global odom2, pub2
    odom2.header = data.header
    odom2.child_frame_id = data.child_frame_id
    odom2.pose.pose = data.pose.pose
    odom2.twist.twist = data.twist.twist

    odom2.pose.covariance[0] = data.pose.covariance[0] * vo_gain
    odom2.pose.covariance[7] = data.pose.covariance[7] * vo_gain
    odom2.pose.covariance[35] = data.pose.covariance[35]  * vo_gain_z
        
    pub2.publish(odom2)
    

pub = rospy.Publisher('/robot_odom_wheels_2', Odometry, queue_size=2)
rospy.Subscriber("/robot_odom_wheels", Odometry, Callback1, queue_size = 1)
pub2 = rospy.Publisher('/rtabmap/odom_2', Odometry, queue_size=2)
rospy.Subscriber("/rtabmap/odom", Odometry, Callback2, queue_size = 1)


rospy.spin()

