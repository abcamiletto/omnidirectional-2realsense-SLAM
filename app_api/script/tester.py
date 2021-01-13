#!/usr/bin/env python


import rospy
import socket
import sys
import json

def create_json(key, sender, command, pace = [0,0,0], error = None):
    dict = {"sender": sender,
            "command": command,
            "pace": pace,
            "error": error}
    json_msg = json.dumps(dict)
    return json_msg


UDP_IP = "192.168.0.100"
UDP_PORT = 3333
MESSAGE = create_json(42,'io','rosnode kill state_machine')
MESSAGE2 = create_json(42,'io','roslaunch state_machine sm_startup.launch use_local_joy:=true')
MESSAGE3 = create_json(42,'io','', pace = [-0.1,0,0])

print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)
print("message: %s" % MESSAGE3)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.sendto(MESSAGE3, (UDP_IP, UDP_PORT))
