#!/usr/bin/env python
# coding: utf-8

######### IMPORT LIBRARIES #########

import rospy
import socket
import numpy as np
from kinematics_matrix import jacobian, driver_constant
from geometry_msgs.msg import Twist
from state_machine.msg import UDPmessage

######### DEFINE "GLOBAL" VARIABLES AND PARAMETERS #########

#init global variable twist 2D
twist_2D = np.zeros(3, np.float32)
#driver output mask
output_mask = driver_constant*np.array([-1.0, 1.0, -1.0, 1.0], np.float32)

motor_write=False
motor_enable=False
lamp_enable=False

# generate enable according to MCU code
def compose_enable_signal(motor_write, motor_enable, lamp_enable):
    output = int(motor_write) + 2*int(motor_enable) + 8*int(lamp_enable)
    _output=np.array(output,np.uint8)
    return _output.tobytes()

# disable bytes (watchdog<=0 --> no incoming msgs from publisher)
disable_output=(np.array([0,0,0,0], np.int16)).tobytes()
disable_bytes = disable_output + compose_enable_signal(False,False,False)

######### WATCHDOG ON /CMD_VEL #########

# watchdog max value (cycles)
watchdog_max = 50;
# watchdog counter init
watchdog_cnt = 0;

######### CUSTOM FUNCTIONS #########

# udp send
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
def send_via_udp(message, sock, UDP_IP = "10.24.4.234", UDP_PORT = 9999):
    sock.sendto(message, (UDP_IP, UDP_PORT))

######### SUBSCRIBER CALLBACKS #########

# subscribe cmd_vel callback
def Callback(data):
    global twist_2D, motor_write, motor_enable, lamp_enable, watchdog_cnt, watchdog_max
    twist_2D = np.array((data.twist.linear.x, data.twist.linear.y, data.twist.angular.z), np.float32)
    motor_write = data.write_motor
    motor_enable = data.motor_enable
    lamp_enable = data.lamp_enable
    watchdog_cnt = watchdog_max

######### ROS NODE, SUBSCRIBERS AND PUBLISHERS INITIALIZATION #########

# ros init
rospy.init_node('udp_controller', anonymous=True)
# ros subscribe to topics
rospy.Subscriber("cmd_vel", UDPmessage, Callback)
# ros set rate
r = rospy.Rate(100)

while not rospy.is_shutdown():
    
    # SEND WHEEL VELOCITY CODE
    
    # from twist to wheel speeds
    wheel_velocity = np.matmul(jacobian, twist_2D)
    
    # driver constant and conversion to int16 and to bytes
    wheel_output = wheel_velocity * output_mask
    wheel_output = (wheel_output.astype(np.int16)).tobytes()
    
    # enable
    enable_output = compose_enable_signal(motor_write, motor_enable, lamp_enable)
    
    # output bytes  (watchdog>0  --> ok incoming msgs from publisher)
    output_bytes = wheel_output + enable_output
    
    # send data logic with watchdog
    if watchdog_cnt>0:
        watchdog_cnt-=1
        send_via_udp(output_bytes, sock)
    else:
        # disable bytes (watchdog<=0 --> no incoming msgs from publisher)
        send_via_udp(disable_bytes, sock)
    
    # ROS SLEEP
    r.sleep()
    

