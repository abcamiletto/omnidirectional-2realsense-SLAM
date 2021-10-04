#!/usr/bin/env python
# coding: utf-8

######### IMPORT LIBRARIES #########

import rospy
import socket
import numpy as np
from geometry_msgs.msg import Twist

######### DEFINE "GLOBAL" VARIABLES AND PARAMETERS #########

#init global variable head_vel
head_vel = np.zeros(2, np.float32)
max_head_vel = 1.5
min_head_vel = 0.0

motor_x_enable=False
motor_y_enable=False

# generate enable according to MCU code
def compose_enable_signal(motor_x_enable, motor_y_enable):
    output = int(motor_x_enable) + 2*int(motor_y_enable)
    _output=np.array(output,np.uint8)
    return _output.tobytes()
    
def filter_input(vel_in, min_vel=min_head_vel, max_vel=max_head_vel):
    new_vel = vel_in
    if vel_in[0] > max_vel: # set maximum value for velocity
        new_vel[0] = max_vel
    elif vel_in[0] < -max_vel:
        new_vel[0] = -max_vel
    elif abs(vel_in[0])<min_vel:
        new_vel[0]=0.0
    if vel_in[1] > max_vel: # set maximum value for velocity
        new_vel[1] = max_vel
    elif vel_in[1] < -max_vel:
        new_vel[1] = -max_vel
    elif abs(vel_in[1])<min_vel:
        new_vel[1]=0.0
    return new_vel

# disable bytes (watchdog<=0 --> no incoming msgs from publisher)
disable_output=(np.array([0,0], np.float32)).tobytes()
disable_bytes = disable_output + compose_enable_signal(False,False)

######### WATCHDOG ON /CMD_VEL #########

# watchdog max value (cycles)
watchdog_max = 50;
# watchdog counter init
watchdog_cnt = 0;

######### CUSTOM FUNCTIONS #########

# udp send
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
def send_via_udp(message, sock, UDP_IP = "10.24.4.20", UDP_PORT = 9999):	# !! put stm address and port
    sock.sendto(message, (UDP_IP, UDP_PORT))

######### SUBSCRIBER CALLBACKS #########

# subscribe cmd_vel callback
def Callback(data):
    global head_vel, motor_x_enable, motor_y_enable, watchdog_cnt
    head_vel = np.array((data.angular.z, -data.angular.y), np.float32)	# !! put "data" type received by receiver & check axis (
    motor_x_enable = True
    motor_y_enable = True
    watchdog_cnt = watchdog_max

######### ROS NODE, SUBSCRIBERS AND PUBLISHERS INITIALIZATION #########

# ros init
rospy.init_node('udp_controller_head', anonymous=True)
# ros subscribe to topics
rospy.Subscriber("/head/twist", Twist, Callback)	# !! put the topic published by UDP receiver
# ros set rate
f = 100.0
dt = 1.0/f # time interval
r = rospy.Rate(f)

freq_taglio=5.0
old_output=np.array([0.0, 0.0], np.float32)
a=(1.0/freq_taglio)/(1.0/freq_taglio+dt)
b=1.0-a

while not rospy.is_shutdown():
    
    # SEND HEAD VELOCITY CODE
    # from position to velocity
    
    '''
    head_vel = (head_ori - old_head_ori) / dt # [rad/s] !! check dim
    old_head_ori = head_ori
    
    
    if abs(head_vel[0]) < 0.2:
        head_vel[0] = 0.0
    else:
        old_head_ori[0] = head_ori[0]
    
    if abs(head_vel[1]) < 0.2:
        head_vel[1] = 0.0
    else:
        old_head_ori[1] = head_ori[1]
    '''
    rospy.loginfo(head_vel)
    
    head_vel_filtered=a*old_output+b*head_vel
    old_output=head_vel_filtered
    
    head_vel_filtered=filter_input(head_vel_filtered)
    '''
    if head_vel_filtered[0] > max_head_vel: # set maximum value for velocity
        head_vel_filtered[0] = max_head_vel
    elif head_vel_filtered[0] < -max_head_vel:
        head_vel_filtered[0] = -max_head_vel
    if head_vel_filtered[1] > max_head_vel: # set maximum value for velocity
        head_vel_filtered[1] = max_head_vel
    elif head_vel_filtered[1] < -max_head_vel:
        head_vel_filtered[1] = -max_head_vel
    '''
    
    
    # conversion to bytes
    head_vel_output = head_vel_filtered.tobytes()      # check if conversion is necessary 
    #head_vel_output = head_vel.tobytes()
    
    # enable
    enable_output = compose_enable_signal(motor_x_enable, motor_y_enable)
    
    # output bytes  (watchdog>0  --> ok incoming msgs from publisher)
    output_bytes = head_vel_output + enable_output
    
    # send data logic with watchdog
    if watchdog_cnt>0:
        watchdog_cnt-=1
        send_via_udp(output_bytes, sock)
    else:
        # disable bytes (watchdog<=0 --> no incoming msgs from publisher)
        send_via_udp(disable_bytes, sock)
    
    # ROS SLEEP
    r.sleep()
