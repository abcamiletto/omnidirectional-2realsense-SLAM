#!/usr/bin/env python
# coding: utf-8

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8


ENABLE = Int8()
ENABLE.data = 6


thu_a_scaling = 255
thu_f_scaling = 255
ind_f_scaling = 255
mid_f_scaling = 255
ril_f_scaling = 255
my_joint_names = ["THU_A", "THU_F", "IND_F", "MID_F", "RIL_F"]
my_command_msg = JointTrajectory(joint_names=my_joint_names)

mypoint0 = JointTrajectoryPoint()
mypoint1 = JointTrajectoryPoint()
mypoint2 = JointTrajectoryPoint()
mypoint3 = JointTrajectoryPoint()
mypoint4 = JointTrajectoryPoint()
mypoint0.positions = [0.0]
mypoint1.positions = [0.0]
mypoint2.positions = [0.0]
mypoint3.positions = [0.0]
mypoint4.positions = [0.0]

my_command_msg.points = [mypoint0, mypoint1, mypoint2, mypoint3, mypoint4]


def limit_ref_values(data_in, max_value=1, min_value=0):
    if data_in > max_value:
        data_in = max_value
    elif data_in < min_value:
        data_in = min_value
    return data_in


def commands_cbk(data):
    global my_command_msg
    my_command_msg.points[1].positions = [thu_a_scaling * limit_ref_values(data.linear.x)]
    my_command_msg.points[2].positions = [thu_f_scaling * limit_ref_values(data.linear.y)]
    my_command_msg.points[3].positions = [ind_f_scaling * limit_ref_values(data.linear.z)]
    my_command_msg.points[4].positions = [mid_f_scaling * limit_ref_values(data.angular.x)]
    my_command_msg.points[0].positions = [ril_f_scaling * limit_ref_values(data.angular.y)]


if __name__ == '__main__':
    rospy.init_node('exos2azzurra', anonymous=True)
    stream_enabler = rospy.Publisher('/streaming_enabler/command', Int8, queue_size=1, latch=True)
    stream_enabler.publish(ENABLE)
    rospy.Subscriber("/azzurra_hand_com", Twist, commands_cbk)
    pub_command = rospy.Publisher('/position_controller/command', JointTrajectory, queue_size=1)
    rate = rospy.Rate(25.0)
    while not rospy.is_shutdown():
        pub_command.publish(my_command_msg)
        rate.sleep()
