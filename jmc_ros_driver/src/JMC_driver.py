#!/usr/bin/python2.7
# coding: utf-8

import numpy as np
import socket
import rospy

class JMC_driver:

    def __init__(self, IP_config=("192.168.0.10",9999,"192.168.0.100",11111), gain=4.0, max_angle=30.0, min_angle=-30.0):
    
        self.micro_IP = IP_config[0]
        self.micro_port = IP_config[1]
        self.personal_IP = IP_config[2]
        self.personal_port = IP_config[3]
        
        # user specified attributes
        
        self.buffer_length = 9
        self.change_mode = np.uint8(0)
        self.send_position = np.float32(0.0)
        self.send_gain = np.float32(gain)
        
        # echo attributes
        
        self.receive_control_mode = np.uint8(0)
        self.receive_position = np.float32(0.0)
        self.receive_gain = np.float32(0.0)
        self.max_angle = np.float32(max_angle)
        self.min_angle = np.float32(min_angle)
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.personal_IP, self.personal_port))
        
        
    def __str__(self):
        return 'smc_driver(' + str(self.micro_IP) + ',' + str(self.micro_port) + ')\n'
        
        
    def __repr__(self):
        print('smc_driver(' + str(self.micro_IP) + ',' + str(self.micro_port) + ')\n')


    def set_buttons(self, button):
        self.change_mode = np.uint8(button)

        
    def set_desired_position(self, dp):
        self.send_position = np.float32(dp)
        if self.send_position > self.max_angle:
            self.send_position = self.max_angle
        elif self.send_position < self.min_angle:
            self.send_position = self.min_angle
        
        
    def set_gain(self, k):
        self.send_gain = np.float32(k)
        
    
    def set_mode(self, mode):
        if self.receive_control_mode != mode:
            self.change_mode = np.uint8(1)
            
        
        
    def send_to_driver(self):
        
        control_mode_button_bytes = self.change_mode.tobytes()
        self.change_mode = np.uint8(0)
        position_bytes = self.send_position.tobytes()
        gain_bytes = self.send_gain.tobytes()
        
        output_bytes = control_mode_button_bytes + position_bytes + gain_bytes

        self.sock.sendto(output_bytes, (self.micro_IP, self.micro_port))
        
        
    def receive_from_driver(self):
        
        data, _ = self.sock.recvfrom(self.buffer_length)
        self.receive_control_mode = np.frombuffer(data, dtype=np.uint8, count = 1, offset = 0)
        self.receive_position = np.frombuffer(data, dtype=np.float32, count = 1, offset = 1)
        self.receive_gain = np.frombuffer(data, dtype=np.float32, count = 1, offset = 5)
        
        
    def driver_echo(self):
        
        print('Current attributes:\n')
        print('Control mode:'+ str(self.receive_control_mode) + '\n')
        print('Position:'+ str(self.receive_position) + '\n')
        print('\n')

    def logging(self):
        rospy.loginfo('mode: {name}\t angle: {position}\t gain: {gain}'.format(name=self.receive_control_mode, position=self.receive_position, gain=self.receive_gain))

