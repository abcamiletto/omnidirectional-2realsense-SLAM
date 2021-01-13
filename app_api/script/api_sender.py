#!/usr/bin/env python3
import rospy
import socket
import sys
import json
import subprocess

rospy.init_node('api_sender')

def create_json(sender, command, value = None):
    dict = {"sender": sender,
            "topic": topic,
            "value": value}
    json_msg = json.dumps(dict)
    return json_msg

UDP_IP = "192.168.0.100"
UDP_PORT = 3333
MESSAGE = create_json('COVID_ROBOT', 'PACE', 1.0)

print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)
print("message: %s" % MESSAGE)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT)) 
