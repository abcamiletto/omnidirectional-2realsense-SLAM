#!/usr/bin/env python

import rospy
import socket
import numpy as np
import threading
from mia_hand_msgs.msg import FingersStrainGauges
from std_msgs.msg import Float64
from std_srvs.srv import Empty, EmptyRequest
from select import select
from queue import Queue


thu_scaling = 255
ind_scaling = 225
mrl_scaling = 255

pub_rate = 25.0

LOCAL_IP = "10.24.4.100"
MIA_PORT = 10200
REMOTE_IP = "10.24.4.35"
REMOTE_PORT = 10250
MIA_SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
MIA_SOCK.bind((LOCAL_IP, MIA_PORT))


def send_via_udp(message,
                 sock=MIA_SOCK,
                 rem_ip=REMOTE_IP,
                 rem_port=REMOTE_PORT):
    sock.sendto(message, (rem_ip, rem_port))


def streaming_cbk(msg):
    send_via_udp(np.array([msg.thu[0], msg.ind[0], msg.mrl[0]], np.float32).tobytes())


def limit_ref_values(data_in, max_value=1, min_value=0):
    if data_in > max_value:
        data_in = max_value
    elif data_in < min_value:
        data_in = min_value
    return data_in


if __name__ == '__main__':
    rospy.init_node('udp_mia')
    sub_str = rospy.Subscriber("/mia/fin_sg", FingersStrainGauges, streaming_cbk)
    pub_mcp1 = rospy.Publisher('/MCP1_position_controller/command', Float64, queue_size=1)
    pub_mcp2 = rospy.Publisher('/MCP2_position_controller/command', Float64, queue_size=1)
    pub_mcp3 = rospy.Publisher('/MCP3_position_controller/command', Float64, queue_size=1)

    thu_ref = Float64(data=1.0)
    ind_ref = Float64(data=31.0)
    mrl_ref = Float64(data=1.0)

    rospy.wait_for_service('/mia/ana_stream_on')
    resp = rospy.ServiceProxy('/mia/ana_stream_on', Empty)(EmptyRequest())

    q = Queue(maxsize=4)

    def receive_routine(s=MIA_SOCK, length=12, timeout=0.04):
        while not rospy.is_shutdown():
            read_fds, _, _ = select([s], [], [], timeout)
            if s in read_fds:
                data, _ = s.recvfrom(length)
                data = np.frombuffer(data, dtype=np.float32, count=3)
                q.put(data)

    mia_com = np.array([1.0, 31.0, 1.0], dtype=np.float32)

    t = threading.Thread(target=receive_routine)
    t.start()

    rate = rospy.Rate(pub_rate)
    while not rospy.is_shutdown():
        if not q.empty():
            mia_com = q.get()
        thu_ref.data = thu_scaling * limit_ref_values(mia_com[0])
        ind_ref.data = 30 + ind_scaling * limit_ref_values(mia_com[1])
        mrl_ref.data = mrl_scaling * limit_ref_values(mia_com[2])
        pub_mcp1.publish(thu_ref)
        pub_mcp2.publish(ind_ref)
        pub_mcp3.publish(mrl_ref)
        rate.sleep()

    t.join()

    print('Shutdown ok')
