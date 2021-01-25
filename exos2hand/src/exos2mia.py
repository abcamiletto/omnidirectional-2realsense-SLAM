#!/usr/bin/env python
# coding: utf-8

import rospy
import numpy as np
from std_msgs.msg import Float64
from mia_hand_msgs.msg import FingersData
from geometry_msgs.msg import Twist

#thu_pos=0
#ind_pos=0
#mrl_pos=0
thu_scaling=255
ind_scaling=225
mrl_scaling=255
thu_ref=Float64(data=1.0)
ind_ref=Float64(data=31.0)
mrl_ref=Float64(data=1.0)
mia_pose=Twist()

def limit_ref_values(data_in, max_value=1, min_value=0):
    if data_in>max_value:
        data_in=max_value
    elif data_in<min_value:
        data_in=min_value
    else:
        pass
    return data_in

def streamingCallback(msg):
    global mia_pose
    mia_pose.linear.x = msg.thu
    mia_pose.linear.y = msg.ind
    mia_pose.linear.z = msg.mrl
    pub_pose.publish(mia_pose)

def commandsCallback(incdata):
    global thu_ref, ind_ref, mrl_ref
    thu_ref.data = thu_scaling*limit_ref_values(incdata.linear.x)
    ind_ref.data = 30+ind_scaling*limit_ref_values(incdata.linear.y)
    mrl_ref.data = mrl_scaling*limit_ref_values(incdata.linear.z)

rospy.init_node('exos2mia', anonymous=True)
sub_str = rospy.Subscriber("/mia/fin_pos", FingersData, streamingCallback)
sub_com = rospy.Subscriber("/mia_hand_com", Twist, commandsCallback)
pub_mcp1 = rospy.Publisher('/MCP1_position_controller/command', Float64, queue_size=2)
pub_mcp2 = rospy.Publisher('/MCP2_position_controller/command', Float64, queue_size=2)
pub_mcp3 = rospy.Publisher('/MCP3_position_controller/command', Float64, queue_size=2)
pub_pose = rospy.Publisher('/mia_pose', Twist, queue_size=2)
rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    pub_mcp1.publish(thu_ref)
    pub_mcp2.publish(ind_ref)
    pub_mcp3.publish(mrl_ref)
    rate.sleep()
        
    

    

    

