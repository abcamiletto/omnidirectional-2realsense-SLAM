#!/usr/bin/env python
# coding: utf-8

import rospy
import socket
import numpy as np
from geometry_msgs.msg import Twist

pedal_tw=Twist()

myrate=100.0
lin_sp_lim=0.2
ang_sp_lim=np.pi/6

local_IP= "10.24.4.100"
pedal_port= 15006

pedal_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
pedal_sock.bind((local_IP, pedal_port))

rospy.init_node('udp_pedal', anonymous=True)
pedal_pub = rospy.Publisher('cmd_vel_pedal', Twist, queue_size=2)
loop_rate=rospy.Rate(myrate)

while not rospy.is_shutdown():
    pedal_data, _ = pedal_sock.recvfrom(12)
    pedal_com = np.frombuffer(pedal_data, dtype=np.float32, count=3)
    pedal_tw.linear.x = lin_sp_lim*pedal_com[0]
    pedal_tw.linear.y = lin_sp_lim*pedal_com[1]
    pedal_tw.angular.z = ang_sp_lim*pedal_com[2]
    pedal_pub.publish(pedal_tw)
    loop_rate.sleep()


