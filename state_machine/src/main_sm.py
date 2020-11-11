#!/usr/bin/env python
# coding: utf-8

######### IMPORT LIBRARIES #########

import rospy
import smach
import smach_ros
import numpy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from state_machine.msg import UDPmessage
from std_msgs.msg import String

######### DEFINE "GLOBAL" VARIABLES AND PARAMETERS #########

r1, l1, Triangle = [0] * 3
state_mode = 0
system_enable = 0
lamp_enable = 0
auto_twist = Twist()
joy_twist = Twist()
empty_twist = Twist()
udp_message = UDPmessage(twist=empty_twist, write_motor=False, motor_enable=False, lamp_enable=False, addon=False)
State_String=String(data='disabled')
state_names = ['disabled','joystick','auto']
watchdog_max=50
watchdog_cnt=watchdog_max

######### SUBSCRIBER CALLBACKS #########

def callback_auto(data):
    global auto_twist
    auto_twist = data
    
def callback_joy(data):
    global joy_twist
    joy_twist = data

def callback(data):
    global r1, l1, Triangle, state_mode, system_enable, lamp_enable, watchdog_cnt, watchdog_max
    watchdog_cnt=watchdog_max
    # # Bluetooth Joystick (Daniele/Nuovo)
    # r1_new = data.buttons[7]
    # l1_new = data.buttons[6]
    # Triangle_new=data.buttons[4]
    # # Joystick XTreme Wireless - remote
    # r1_new = data.buttons[5]
    # l1_new = data.buttons[4]
    # Triangle_new=data.buttons[2]
    # Joystick Xtreme only cable - remote
    r1_new = data.buttons[5]
    l1_new = data.buttons[4]
    Triangle_new=data.buttons[0]
    if r1_new > r1:
        state_mode = (state_mode + 1) % 2
    if l1_new > l1:
        system_enable = (system_enable + 1) % 2
    if Triangle_new > Triangle:
        lamp_enable = (lamp_enable + 1) % 2
    r1 = r1_new
    l1 = l1_new
    Triangle = Triangle_new

######### CUSTOM FUNCTIONS #########

def state_decider():
    global watchdog_cnt, state_names, lamp_enable, system_enable, state_mode
    watchdog_cnt-=1
    if watchdog_cnt<0:
        lamp_enable=0
        system_enable=0
        state_mode=0
        return state_names[0]
    else:
        if system_enable != 1:
            return state_names[0]
        return state_names[state_mode + 1]

######### DEFINE STATES #########
	
# define state Disabled
class Disabled(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['joystick','auto'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Disabled')
        State_String='Executing state DISABLED'
        stringpub.publish(State_String)
        rate=rospy.Rate(100)
        while state_decider() == 'disabled' and not rospy.is_shutdown():
            udp_message.twist = empty_twist
            udp_message.write_motor=False
            udp_message.motor_enable=False
            udp_message.lamp_enable=False
            udp_message.addon=False
            pub.publish(udp_message)
            rate.sleep()
        return state_decider()


# define state Joystick
class Joystick(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['disabled', 'auto'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Joystick')
        State_String='Executing state JOYSTICK'
        stringpub.publish(State_String)
        rate=rospy.Rate(100)
        while state_decider() == 'joystick' and not rospy.is_shutdown():
            udp_message.twist = joy_twist
            udp_message.write_motor=True
            udp_message.motor_enable=True
            udp_message.lamp_enable=lamp_enable
            udp_message.addon=False
            pub.publish(udp_message)
            rate.sleep()
        return state_decider()
        
        
# define state Auto
class Auto(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['disabled', 'joystick'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Auto')
        State_String='Executing state AUTO'
        stringpub.publish(State_String)
        rate=rospy.Rate(100)
        while state_decider() == 'auto' and not rospy.is_shutdown():
            udp_message.twist = auto_twist
            udp_message.write_motor=True
            udp_message.motor_enable=True
            udp_message.lamp_enable=lamp_enable
            udp_message.addon=False
            pub.publish(udp_message)
            rate.sleep()
        return state_decider()      

######### NODE INITIALIZATION #########

rospy.init_node('state_machine', anonymous=True)
rospy.Subscriber("joy", Joy, callback)
rospy.Subscriber("cmd_vel_joy", Twist, callback_joy)
rospy.Subscriber("cmd_vel_auto", Twist, callback_auto)
pub = rospy.Publisher('cmd_vel', UDPmessage, queue_size=10)
stringpub = rospy.Publisher('state_string', String, queue_size=1)

######### SMACH INITIALIZATION #########

# Create a SMACH state machine
sm = smach.StateMachine(outcomes=[])

# Open the container
with sm:
    # Add states to the container
    smach.StateMachine.add('DISABLED', Disabled(), 
                           transitions={'joystick':'JOYSTICK', 
                                        'auto':'AUTO'})
    smach.StateMachine.add('JOYSTICK', Joystick(), 
                           transitions={'disabled':'DISABLED', 
                                        'auto':'AUTO'})
    smach.StateMachine.add('AUTO', Auto(), 
                           transitions={'disabled':'DISABLED', 
                                        'joystick':'JOYSTICK'})

# Execute SMACH plan
outcome = sm.execute()

	
