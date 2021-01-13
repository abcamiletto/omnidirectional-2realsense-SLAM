#!/usr/bin/env python3
import rospy
import socket
import sys
import json
import subprocess

rospy.init_node('api_receiver')

def deserialize_json(json_msg):
    return json.loads(json_msg)

UDP_IP = "192.168.0.100"
UDP_PORT = 3333

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

r = rospy.Rate(50) #Hz
while not rospy.is_shutdown():
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    data = deserialize_json(data)
    if data['command']:
        subprocess.Popen(data['command'], shell=True)
    if not data['pace'] == [0,0,0]:
        cmd_vel = "rostopic pub /cmd_vel_auto geometry_msgs/Twist '{{linear:[{0},{1},0], angular:[0,0,{2}]}}'".format(*data['pace'])
        subprocess.Popen(cmd_vel, shell=True)
    r.sleep()
