#!/usr/bin/env python
# coding: utf-8

import rospy
import socket
import numpy as np
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory

azzurra_tw=Twist()

myrate=100.0

local_IP = "10.24.4.100"
azzurra_port = 20200
remote_IP = "10.24.4.35"
remote_port = 20250
tension=np.array([2.0,1.0,-1.0,-2.0,0.0], np.float32)
azzurra_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
azzurra_sock.bind((local_IP, azzurra_port))

def send_via_udp(message, sock=azzurra_sock, rem_IP = remote_IP, rem_port=remote_port):
    sock.sendto(message, (rem_IP, rem_port))

#def callback_pos(data):
#    global position
#    etc etc
    
def callback_ten(data):
    global tension
    tension=np.array(data.points[0].effort, np.float32)

rospy.init_node('udp_azzurra', anonymous=True)
azzurra_pub = rospy.Publisher('/azzurra_hand_com', Twist, queue_size=2)
#rospy.Subscriber("/azzurra_streaming/positions", Twist, callback_pos)
rospy.Subscriber("/streaming/state", JointTrajectory, callback_ten)
loop_rate=rospy.Rate(myrate)

while not rospy.is_shutdown():
    azzurra_data, _ = azzurra_sock.recvfrom(20)
    azzurra_com = np.frombuffer(azzurra_data, dtype=np.float32, count=5)
    azzurra_tw.linear.x = azzurra_com[0]
    azzurra_tw.linear.y = azzurra_com[1]
    azzurra_tw.linear.z = azzurra_com[2]
    azzurra_tw.angular.x = azzurra_com[3]
    azzurra_tw.angular.y = azzurra_com[4]
    azzurra_pub.publish(azzurra_tw)
    udp_message=tension.tobytes()
    send_via_udp(udp_message)
    loop_rate.sleep()

