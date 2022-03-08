#!/usr/bin/env python
# coding: utf-8

import rospy
from std_msgs.msg import Float64
from mia_hand_msgs.msg import FingersData
from geometry_msgs.msg import Twist


thu_scaling = 255
ind_scaling = 225
mrl_scaling = 255


def limit_ref_values(data_in, max_value=1, min_value=0):
    if data_in > max_value:
        data_in = max_value
    elif data_in < min_value:
        data_in = min_value
    return data_in


if __name__ == '__main__':
    rospy.init_node('exos2mia', anonymous=True)

    thu_ref = Float64(data=1.0)
    ind_ref = Float64(data=31.0)
    mrl_ref = Float64(data=1.0)

    pub_mcp1 = rospy.Publisher('/MCP1_position_controller/command', Float64, queue_size=1)
    pub_mcp2 = rospy.Publisher('/MCP2_position_controller/command', Float64, queue_size=1)
    pub_mcp3 = rospy.Publisher('/MCP3_position_controller/command', Float64, queue_size=1)
    pub_pose = rospy.Publisher('/mia_pose', Twist, queue_size=1)

    def streaming_cbk(msg: FingersData):
        mia_pose = Twist()
        mia_pose.linear.x = msg.thu
        mia_pose.linear.y = msg.ind
        mia_pose.linear.z = msg.mrl
        pub_pose.publish(mia_pose)

    def commands_cbk(msg: Twist):
        global thu_ref, ind_ref, mrl_ref
        thu_ref.data = thu_scaling * limit_ref_values(msg.linear.x)
        ind_ref.data = 30 + ind_scaling * limit_ref_values(msg.linear.y)
        mrl_ref.data = mrl_scaling * limit_ref_values(msg.linear.z)

    sub_str = rospy.Subscriber("/mia/fin_sg", FingersData, streaming_cbk)
    sub_com = rospy.Subscriber("/mia_hand_com", Twist, commands_cbk)
    rate = rospy.Rate(25.0)
    while not rospy.is_shutdown():
        pub_mcp1.publish(thu_ref)
        pub_mcp2.publish(ind_ref)
        pub_mcp3.publish(mrl_ref)
        rate.sleep()

        
    

    

    

