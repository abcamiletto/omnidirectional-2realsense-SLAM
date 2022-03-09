#!/usr/bin/env python

import rospy
import socket
import numpy as np
from std_msgs.msg import Int8
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# from threading import Thread
# from library import receive_data

pub_rate = 25.0

LOCAL_IP = "10.24.4.100"
AZZURRA_PORT = 20200
REMOTE_IP = "10.24.4.35"
REMOTE_PORT = 20250
AZZURRA_SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
AZZURRA_SOCK.bind((LOCAL_IP, AZZURRA_PORT))


def send_via_udp(message,
                 sock=AZZURRA_SOCK,
                 rem_ip=REMOTE_IP,
                 rem_port=REMOTE_PORT):
    sock.sendto(message, (rem_ip, rem_port))


ENABLE = Int8()
ENABLE.data = 6

thu_a_scaling = 255
thu_f_scaling = 255
ind_f_scaling = 255
mid_f_scaling = 255
ril_f_scaling = 255

my_joint_names = ["THU_A", "THU_F", "IND_F", "MID_F", "RIL_F"]


def limit_ref_values(data_in, max_value=1, min_value=0):
    if data_in > max_value:
        data_in = max_value
    elif data_in < min_value:
        data_in = min_value
    return data_in


def callback_ten(msg):
    tension = np.array(msg.points[0].effort, np.float32)
    udp_message = tension.tobytes()
    send_via_udp(udp_message)


if __name__ == '__main__':
    rospy.init_node('udp_azzurra')
    pub_command = rospy.Publisher('/position_controller/command', JointTrajectory, queue_size=1)
    rospy.Subscriber("/streaming/state", JointTrajectory, callback_ten)
    stream_enabler = rospy.Publisher('/streaming_enabler/command', Int8, queue_size=1, latch=True)
    stream_enabler.publish(ENABLE)

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

    azzurra_com = np.frombuffer(bytearray(20), dtype=np.float32, count=5)

    # def data_reception(sock=AZZURRA_SOCK, buffer_len=20):
    #     global azzurra_com
    #     local_rate = rospy.Rate(25.0)
    #     while not rospy.is_shutdown():
    #         data = receive_data(sock, buffer_len)
    #         if data is not None:
    #             azzurra_com = np.frombuffer(data, dtype=np.float32, count=5)
    #         local_rate.sleep()
    #
    # reception_thread = Thread(target=data_reception, name="ReceiveData")
    # reception_thread.start()

    rate = rospy.Rate(pub_rate)
    while not rospy.is_shutdown():
        data, _ = AZZURRA_SOCK.recvfrom(20)
        azzurra_com = np.frombuffer(data, dtype=np.float32, count=5)
        rospy.loginfo(azzurra_com)
        my_command_msg.points[1].positions = [thu_a_scaling * limit_ref_values(azzurra_com[0])]
        my_command_msg.points[2].positions = [thu_f_scaling * limit_ref_values(azzurra_com[1])]
        my_command_msg.points[3].positions = [ind_f_scaling * limit_ref_values(azzurra_com[2])]
        my_command_msg.points[4].positions = [mid_f_scaling * limit_ref_values(azzurra_com[3])]
        my_command_msg.points[0].positions = [ril_f_scaling * limit_ref_values(azzurra_com[4])]
        pub_command.publish(my_command_msg)
        rate.sleep()

    # reception_thread.join(timeout=5.0)

    print('Shutdown ok')
