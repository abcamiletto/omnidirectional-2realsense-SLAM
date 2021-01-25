#!/usr/bin/env python
# coding: utf-8

import rospy
import socket
import numpy as np
from geometry_msgs.msg import Twist

mia_tw=Twist()

myrate=100.0

local_IP= "10.24.4.100"
mia_port= 20500
remote_IP = "10.24.4.35"
remote_port = 15000

position=np.array([0.0,0.0,0.0],np.float32)

mia_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
mia_sock.bind((local_IP, mia_port))

def send_via_udp(message, sock=mia_sock, rem_IP = remote_IP, rem_port=remote_port):
    sock.sendto(message, (rem_IP, rem_port))

def callback_pos(data):
    global position
    position=np.array([data.linear.x, data.linear.y, data.linear.z], np.float32)

rospy.init_node('udp_mia', anonymous=True)
mia_pub = rospy.Publisher('/mia_hand_com', Twist, queue_size=2)
rospy.Subscriber("/mia_pose", Twist, callback_pos)
loop_rate=rospy.Rate(myrate)

while not rospy.is_shutdown():
    mia_data, _ = mia_sock.recvfrom(12)
    mia_com = np.frombuffer(mia_data, dtype=np.float32, count=3)
    mia_tw.linear.x = mia_com[0]
    mia_tw.linear.y = mia_com[1]
    mia_tw.linear.z = mia_com[2]
    mia_pub.publish(mia_tw)
    udp_message=position.tobytes()
    send_via_udp(udp_message)
    loop_rate.sleep()


