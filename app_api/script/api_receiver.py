#!/usr/bin/env python3
import rospy
import socket
import sys
import json
import subprocess

def deserialize_json(json_msg):
    return json.loads(json_msg)

UDP_IP = "192.168.0.100"
UDP_PORT = 3333

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

r = rospy.rate(50) #Hz
while not rospy.is_shutdown():
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    data = deserialize_json(data)
    subprocess.Popen(data['command'], shell=True)
    r.sleep()
