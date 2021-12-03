#!/usr/bin/python2.7
# coding: utf-8

######### IMPORT LIBRARIES #########

import rospy
import socket
import numpy as np
import JMC_driver as jmc
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Bool
from connection_params import avatar_IP_Config

######### DEFINE "GLOBAL" VARIABLES AND PARAMETERS #########

driver = jmc.JMC_driver(IP_config=avatar_IP_Config)
print(driver)
enable = Bool(0)

######### SUBSCRIBER CALLBACKS #########

def joystick_callback(data):
    driver.set_buttons(data.buttons[5])
    
def position_callback(pos):
    driver.set_desired_position(180.0*pos.data)
    
def gain_callback(k):
    driver.set_gain(k.data)
    
def mode_callback(enable):
    driver.set_mode(enable.data)

######### ROS NODE, SUBSCRIBERS AND PUBLISHERS INITIALIZATION #########

# ros init
rospy.init_node('Data_exchange', anonymous=True)
# ros subscribe to topics
rospy.Subscriber("joy", Joy, joystick_callback)
rospy.Subscriber("head_yaw", Float64, position_callback)
rospy.Subscriber("set_gain", Float64, gain_callback)
rospy.Subscriber("set_mode", Bool, mode_callback)
# ros set rate
r = rospy.Rate(100)

print_counter = 0

while not rospy.is_shutdown():	
    
    driver.send_to_driver()
    driver.receive_from_driver()
    
    print_counter += 1
    if print_counter > 10:
        driver.logging()
        print_counter = 0
        
    # ROS SLEEP
    r.sleep()
