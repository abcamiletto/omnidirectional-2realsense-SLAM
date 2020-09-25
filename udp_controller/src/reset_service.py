#!/usr/bin/env python
# coding: utf-8

######### IMPORT LIBRARIES #########

import rospy
from udp_controller.srv import *

######### DEFINE "GLOBAL" VARIABLES AND PARAMETERS #########
RESET_FLAG = False
# reset_odom = reset_odom()

######### WATCHDOG ON /CMD_VEL #########


######### CUSTOM FUNCTIONS #########
def set_reset_flag():
    if RESET_FLAG == 0:
        return False
    elif RESET_FLAG == 1:
        return True

def call_reset_odom():
    # global reset_odom
    rospy.wait_for_service('reset_msg')
    try:
        reset_service = rospy.ServiceProxy('reset_msg', reset_odom)
        reset_service(True)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

######### SUBSCRIBER CALLBACKS #########


######### ROS NODE, SUBSCRIBERS AND PUBLISHERS INITIALIZATION #########

# ros init
rospy.init_node('reset_service')

# ros set rate
r = rospy.Rate(100)

counter = 0

while not rospy.is_shutdown():

    counter += 1
    if counter > 500: # dopo 5 secondi
        call_reset_odom()
        print("odometry reset")
        counter = 0
    else:
        RESET_FLAG = 0

    # ROS SLEEP
    r.sleep()
    

