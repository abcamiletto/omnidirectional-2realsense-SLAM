#!/usr/bin/env python
# coding: utf-8

######### IMPORT LIBRARIES #########

import rospy
import socket
import numpy as np
from geometry_msgs.msg import Twist
from kinematics_matrix import inv_jacobian, encoder_constant
from udp_controller.srv import reset_odom, reset_odomResponse
import tf

######### DEFINE "GLOBAL" VARIABLES AND PARAMETERS #########

#Pedalboard_command
Pedal_twist=Twist()
#Linear_speed_limit
Lin_speed_limit=0.3
#Angular_speed_limit
Ang_speed_limit=np.pi/4
#init encoder raw count value
wh_speeds_enc = np.zeros(4, np.float32)
#encoder input mask
input_mask = encoder_constant*np.array([-1.0, 1.0, -1.0, 1.0], np.float32)
# relative velocity and angular speed
v_rel = np.array([0.0, 0.0, 0.0], np.float32)   # [m/s], [m/s], [1/s]
# odometry values
odom = np.array([0.0, 0.0, 0.0], np.float32)    # [m], [m], [rad]
old_odom = np.array([0.0, 0.0, 0.0], np.float32)    # [m], [m], [rad]
# node rate and time step
RATE = 100.0    # [Hz]
dt = 1.0/RATE   # [s]
# reset boolean: if 0 update current odometry, if 1 reset x, y, teta to 0
RESET_FLAG = False

######### CUSTOM FUNCTIONS #########

# broadcaster of the odometry ad tf message
def handle_robot_pose(_odom):
    br = tf.TransformBroadcaster()  # create a tf
    br.sendTransform([_odom[0], _odom[1], 0.0], # load position
                     tf.transformations.quaternion_from_euler(0, 0, _odom[2]),  # load quaternion
                     rospy.Time.now(),  # send corrent time
                     "base_link",    # to
                     "odom")         # from

def reset_robot_pose(req):
    if req.reset == 1:
        global odom, old_odom
        # save the odometry before the reset
        old_odom = odom
        # reset odometry
        odom = np.array([0.0, 0.0, 0.0], np.float32)  # [m], [m], [rad]
        return reset_odomResponse(True)

######### SET LOCAL SOCKET IP ADDRESS AND UDP PORT #########
personal_IP = "192.168.0.100"
personal_port = 11111
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((personal_IP, personal_port))

personal_IP2= "10.24.4.100"
personal_port2 = 15006
sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock2.bind((personal_IP2, personal_port2))

# ros init
rospy.init_node('udp_receiver', anonymous=True)
pub = rospy.Publisher('cmd_vel_pedal', Twist, queue_size=10)
# ros set rate
r = rospy.Rate(RATE)

print_counter = 0
print_counter2 = 0

# create the service
ser = rospy.Service('reset_msg', reset_odom, reset_robot_pose)

while not rospy.is_shutdown():

    # RECEIVE WHEEL VELOCITY CODE
    
    # Receive from pedalboard
    data_pedalboard, _ = sock2.recvfrom(12)
    Pedalboard_commands = np.frombuffer(data_pedalboard, dtype=np.float32, count=3)
    Pedal_twist.linear.x = Lin_speed_limit*Pedalboard_commands[0]
    Pedal_twist.linear.y = Lin_speed_limit*Pedalboard_commands[1]
    Pedal_twist.angular.z = Ang_speed_limit*Pedalboard_commands[2]
    pub.publish(Pedal_twist)
    
    # Receive incoming packet
    data, _ = sock.recvfrom(18)   #receive 18 bytes
    # interpret first 16 bytes as wheel velocities
    wh_speeds_enc = np.frombuffer(data, dtype=np.float32, count=4)*input_mask
    # interpret last 2 bytes as robot state
    enable_input = np.frombuffer(data, dtype=np.uint8, count=2, offset=16)

    '''
    # Print values on terminal
    print_counter += 1
    if print_counter > 100:
        print(Pedalboard_commands)
        print_counter = 0
    '''

    #   CALCULATE ODOMETRY

    # calculate linear and angular relative speed of the robot
    v_rel = np.matmul(inv_jacobian, wh_speeds_enc)

    # update odom_positions
    odom += v_rel * dt

    '''
    print_counter2 += 1
    if print_counter2 > 100:
        print("vel = ", v_rel)
        print("odom = ", odom * np.array([1.0, 1.0, 180.0/np.pi]))
        print_counter2 = 0
    '''

    # publish tf message
    handle_robot_pose(odom)

    # graph()

    # ROS SLEEP
    r.sleep()

