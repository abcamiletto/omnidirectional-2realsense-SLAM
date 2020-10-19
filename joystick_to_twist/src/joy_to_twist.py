#!/usr/bin/env python
# coding: utf-8
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from math import sqrt
import numpy as np

#y, x, theta = [0] * 3
max_vel = 0.2
max_ang = np.pi/8

######### WATCHDOG ON /joy #########
# watchdog max value (cycles)
watchdog_max = 50;
# watchdog counter init
watchdog_cnt = 0;

def callback(data):
    global max_vel, max_ang, watchdog_cnt, watchdog_max
    twist = Twist()
    try:
        y = data.axes[0]
        x = data.axes[1]
        norm = sqrt(x**2+y**2)
        if norm > 1:
            y = y / norm
            x = x / norm
        x = max_vel*x
        y = max_vel*y
        theta = data.axes[2] * max_ang
        twist.linear.x = x
        twist.linear.y = y
        twist.angular.z = theta
    except:
        print('PDPDPD')
    
    watchdog_cnt = watchdog_max
    
    pub.publish(twist)

rospy.init_node('joy_node', anonymous=True)
rospy.Subscriber("joy", Joy, callback)
pub = rospy.Publisher('cmd_vel_joy', Twist, queue_size=10)

r = rospy.Rate(100)
while not rospy.is_shutdown():
    if watchdog_cnt>0:
        watchdog_cnt-=1
    else:
        pub.publish(Twist())
    r.sleep()
