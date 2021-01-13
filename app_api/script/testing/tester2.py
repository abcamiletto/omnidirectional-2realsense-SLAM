#!/usr/bin/env python


import rospy
import socket
import sys
import json

def create_json(key, sender, command, pace = [0,0]):
    dict = {"key": key,
            "sender": sender,
            "command": command,
            "pace": pace}
    json_msg = json.dumps(dict)
    return json_msg


UDP_IP = "192.168.0.100"
UDP_PORT = 3333
MESSAGE = create_json(42,'io','rosnode kill state_machine')
MESSAGE2 = create_json(42,'io','rosrun webcam_publ web_pub.py')

print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)
print("message: %s" % MESSAGE)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
