#!/usr/bin/env python
# coding: utf-8

######### IMPORT LIBRARIES #########

import rospy
import socket
import numpy as np
from kinematics_matrix import inv_jacobian, encoder_constant
from geometry_msgs.msg import Twist

######### DEFINE "GLOBAL" VARIABLES AND PARAMETERS #########

#init encoder raw count value
wh_speeds_enc = np.zeros(4, np.float32)
#encoder input mask
input_mask = encoder_constant*np.array([-1.0, 1.0, -1.0, 1.0], np.float32)
print_counter=0

######### SET LOCAL SOCKET IP ADDRESS AND UDP PORT #########
personal_IP = "192.168.0.100"
personal_port = 11111
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((personal_IP, personal_port))

rospy.init_node('udp_receive')
r=rospy.Rate(100)

while not rospy.is_shutdown():

    # RECEIVE WHEEL VELOCITY CODE
    
    # Receive incoming packet
    data, _ = sock.recvfrom(18)   #receive 18 bytes
    # interpret first 16 bytes as wheel velocities
    wh_speeds_enc = np.frombuffer(data, dtype=np.float32, count=4)*input_mask
    # interpret last 2 bytes as robot state
    enable_input = np.frombuffer(data, dtype=np.uint8, count=2, offset=16)
    
    # Print values on terminal
    print_counter += 1
    if print_counter > 100:        
        print(wh_speeds_enc*30.0/np.pi)
        print(enable_input)
        print_counter = 0
    
    r.sleep()

