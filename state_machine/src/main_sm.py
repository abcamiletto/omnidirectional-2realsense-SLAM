#!/usr/bin/env python
# coding: utf-8

######### IMPORT LIBRARIES #########

import rospy
import smach
import smach_ros
import numpy
import roslaunch
import rospkg
import math
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from state_machine.msg import UDPmessage
from std_msgs.msg import String
from std_msgs.msg import Float32

######### DEFINE "GLOBAL" VARIABLES AND PARAMETERS #########
rate_hz = 20 #control rate
r1, l1, Triangle = [0] * 3
state_mode = 0
system_enable = 0
lamp_enable = 0
#parameters from parameter server (written in the launch file?)
max_lin_vel = rospy.get_param('/state_machine/max_lin_vel',0.5)
min_lin_vel = rospy.get_param('/state_machine/min_lin_vel',-0.2)
max_ang_vel = rospy.get_param('/state_machine/max_ang_vel',0.2)
max_lin_acc = rospy.get_param('/state_machine/max_ang_vel',0.15) #lin_vel/secondi
max_ang_acc = rospy.get_param('/state_machine/max_ang_acc',0.3) #lin_vel/secondi
robot_model = rospy.get_param('/state_machine/robot_model', 1) #robot model: 1 = robocovid PRO 2 = robocovid Light
vel_reduction_coeff = rospy.get_param('/state_machine/vel_reduction_coeff',0.2)
local_joy=rospy.get_param('state_machine/local_joy',False)
#supervised_twist = Twist()
auto_twist = Twist()
joy_twist = Twist()
pedal_twist = Twist()
empty_twist = Twist()
cur_twist = Twist() #used for acceleration smoothing
output_twist = Twist() #publishes on cmd_vel with the same content of UDPmessage
udp_message = UDPmessage(twist=empty_twist, write_motor=False, motor_enable=False, lamp_enable=False, addon=False)
State_String=String(data='disabled')
state_names = ['disabled','joystick','pedal','auto']
watchdog_max=100
watchdog_cnt=watchdog_max
cnt_log_dist = 10

dist_coeff = 0.0
### Variables for auto-launching other nodes and launchfiles (i.e. slam, mapping, stati tf references)


# class launchManager:

#     flag_running = 0
#     obj_launch = None
#     launchfile_path = ""

#     def __init__(self, path):
#             launchManager.flag_running = 0
#             rospack = rospkg.RosPack()
#             launchManager.launchfile_path = path

#     def enable(self):
#         if(launchManager.flag_running == 0):
#             #launching static tf map->odom to visualize lidar points in rviz without localization
    
#             #rospy.init_node('en_Mapping', anonymous=True)
#             uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
#             roslaunch.configure_logging(uuid)
#             path="/home/pluto/catkin_ws_robolight/src/robolight/state_machine/launch/static_map_to_baselink.launch"
#             launchManager.obj_launch = roslaunch.parent.ROSLaunchParent(uuid, [launchManager.launchfile_path])
#             launchManager.obj_launch.start()
#             rospy.loginfo(launchManager.launchfile_path+" started")
#             launchManager.flag_running = 1

#     def disable(self):
#         if(launchManager.flag_running == 1):  
#             launchManager.obj_launch.shutdown()
#             rospy.loginfo(launchManager.launchfile_path+" shutdown")
#             launchManager.flag_running = 0

#launch_tf = launchManager(rospy.get_param('/state_machine/task1_fullpath',"/home/pluto/catkin_ws_robolight/src/robolight/state_machine/launch/static_map_to_baselink.launch"))
# abilita per lanciare e disabilitare da script la tf static map->baselink

def limitAcceleration(cur, ref, max_acc, dt):
    delta = ref - cur
    d_max = max_acc*dt
    if(delta >= 0):
        if(delta >= d_max):
            cur = cur+d_max
        else:
            cur = ref
    else:
        if(delta <= -d_max):
            cur = cur-d_max
        else:
            cur = ref
    return cur

def limitTwist(in_twist):
    global cur_twist, max_lin_vel, min_lin_vel, max_ang_vel, max_lin_acc, max_ang_acc, rate_hz
    ref_twist = in_twist
    
    #SATURATION
    
    if(robot_model == 1): #1=RobocovidPRO
        linear_vel_module = math.sqrt(in_twist.linear.x*in_twist.linear.x + in_twist.linear.y*in_twist.linear.y)
    
        if(linear_vel_module > max_lin_vel):
            scaling = max_lin_vel/linear_vel_module
            ref_twist.linear.x = in_twist.linear.x*scaling
            ref_twist.linear.y = in_twist.linear.y*scaling
    elif(robot_model == 2): #1=RobocovidLight    
        ref_twist.linear.x = in_twist.linear.x
        if ref_twist.linear.x > max_lin_vel:
                ref_twist.linear.x = max_lin_vel
        if ref_twist.linear.x < min_lin_vel:
                ref_twist.linear.x = min_lin_vel
    else:
        rospy.logerr("NO VALID VALUE FOR robot_model")
            
            
    ref_twist.angular.z = in_twist.angular.z
    if ref_twist.angular.z > max_ang_vel:
            ref_twist.angular.z = max_ang_vel
    if ref_twist.angular.z < -max_ang_vel:
            ref_twist.angular.z = -max_ang_vel
    
    #ACCELERATION
    sampletime = 1.0/rate_hz
    cur_twist.linear.x = limitAcceleration(cur_twist.linear.x, ref_twist.linear.x, max_lin_acc, sampletime)
    cur_twist.linear.y = limitAcceleration(cur_twist.linear.y, ref_twist.linear.y, max_lin_acc, sampletime)
    
    cur_twist.angular.z = limitAcceleration(cur_twist.angular.z, ref_twist.angular.z, max_ang_acc, sampletime)
    #print 'cur x,z: ', cur_twist.linear.x,' ',cur_twist.angular.z 
    return cur_twist
    
######### SUBSCRIBER CALLBACKS #########

#def callback_lidar(data):
#    global supervised_twist, dist_coeff, vel_reduction_coeff
#    dist_coeff = data.data
#    supervised_twist=joy_twist
#    
#    if (supervised_twist.linear.x>=0):
#        if (dist_coeff<vel_reduction_coeff):         
#            supervised_twist.linear.x = 0
#        else:
#            supervised_twist.linear.x = dist_coeff*supervised_twist.linear.x  

# Daniele: DA RIVEDERE LA LOGICA (Qui ora semplificata) saturare le velocità è pericoloso se non abbiamo visibilità laterale o dietro: 
# appena esce da un ostacolo frontale, il riferimento torna non saturato ed accelera all'indietro in modo incontrollato    
# Meglio allora limitare sempre velocità indietro o rotazione sul posto

        #if(abs(supervised_twist.angular.z)>vel_reduction_coeff)
        #        supervised_twist.angular.z = vel_reduction_coeff
    #else:
        #if (supervised_twist.linear.x>=0):
            

def callback_auto(data):
    global auto_twist, dist_coeff, vel_reduction_coeff
    auto_twist = data
    if (auto_twist.linear.x>=0):
        if (dist_coeff<vel_reduction_coeff):         
        
            auto_twist.linear.x = 0
        else:
            auto_twist.linear.x = dist_coeff*auto_twist.linear.x  

    
def callback_joy(data):    # perché variabili globali?
    global joy_twist
    joy_twist = data
    
def callback_pedal(data):
    global pedal_twist
    pedal_twist = data    

def callback(data):
    global r1, l1, Triangle, state_mode, system_enable, lamp_enable, watchdog_cnt
    watchdog_cnt=watchdog_max
    # Remote Joystick Config
    if local_joy:
        r1_new = data.buttons[5]
        l1_new = data.buttons[4]
        Triangle_new=data.buttons[3]
    else:
        r1_new = data.buttons[5]
        l1_new = data.buttons[4]
        Triangle_new=data.buttons[2]
    if r1_new > r1:
        state_mode = (state_mode + 1) % 3
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
        #lamp_enable=0
        #system_enable=0
        #state_mode=0
        return state_names[0]
    else:
        if system_enable != 1:
            return state_names[0]
        return state_names[state_mode + 1]

######### DEFINE STATES #########
	
# define state Disabled
class Disabled(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['joystick','pedal','auto'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Disabled')
        State_String='Executing state DISABLED'
        stringpub.publish(State_String)
        rate=rospy.Rate(rate_hz)

        while state_decider() == 'disabled' and not rospy.is_shutdown():
            udp_message.twist = empty_twist
            udp_message.write_motor=False
            udp_message.motor_enable=False
            udp_message.lamp_enable=lamp_enable
            udp_message.addon=False
            pub_udpcmd.publish(udp_message)
            output_twist = udp_message.twist
            cur_twist = empty_twist
            pub_cmdvel.publish(output_twist)
            rate.sleep()
        return state_decider()


# define state Joystick
class Joystick(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['disabled', 'pedal'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Joystick')
        State_String='Executing state JOYSTICK'
        #launch_tf.enable() #launch static tf between map and baselink to visualize scans in rviz
        stringpub.publish(State_String)
        rate=rospy.Rate(rate_hz)
        cnt=0
        while state_decider() == 'joystick' and not rospy.is_shutdown():
            output_twist = limitTwist(joy_twist)
            if cnt<10:
                udp_message.write_motor=True
            else:
                udp_message.write_motor=True
            udp_message.twist = output_twist
            udp_message.motor_enable=True
            udp_message.lamp_enable=lamp_enable
            udp_message.addon=False
            pub_udpcmd.publish(udp_message)
            pub_cmdvel.publish(output_twist)
            cnt+=1
            rate.sleep()
        #launch_tf.disable()
        return state_decider()
		
# define state pedal
class Pedal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['disabled', 'auto'])
    
    def execute(self, userdata):
        rospy.loginfo('Executing state Pedal')
        State_String='Executing state PEDAL'
        stringpub.publish(State_String)
        rate=rospy.Rate(rate_hz)
        cnt=0
       	while state_decider() == 'pedal' and not rospy.is_shutdown():
            output_twist = limitTwist(pedal_twist)
            if cnt<10:
                udp_message.write_motor=True
            else:
                udp_message.write_motor=True
            udp_message.twist = output_twist
            udp_message.motor_enable=True
            udp_message.lamp_enable=lamp_enable
            udp_message.addon=False
            pub_udpcmd.publish(udp_message)
            pub_cmdvel.publish(output_twist)
            cnt+=1
            rate.sleep()
        return state_decider()
        
# define state Auto
class Auto(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['disabled', 'joystick'])

    def execute(self, userdata):
        global cnt_log_dist
        rospy.loginfo('Executing state Auto')
        State_String='Executing state AUTO'
        stringpub.publish(State_String)
        rate=rospy.Rate(rate_hz)
        cnt=0
        while state_decider() == 'auto' and not rospy.is_shutdown():
            output_twist = limitTwist(auto_twist)
            if cnt<10:
                udp_message.write_motor=True
            else:
                udp_message.write_motor=True

            cnt_log_dist = cnt_log_dist-1
            if(cnt_log_dist<0):
                cnt_log_dist = 10
                #print "distance coefficient: " +str(dist_coeff)
            udp_message.twist = output_twist
            udp_message.motor_enable=True
            udp_message.lamp_enable=lamp_enable
            udp_message.addon=False
            pub_udpcmd.publish(udp_message)
            pub_cmdvel.publish(output_twist)
            cnt+=1
            rate.sleep()
        return state_decider()      

######### NODE INITIALIZATION #########

rospy.init_node('state_machine', anonymous=True)
rospy.Subscriber("joy", Joy, callback)
rospy.Subscriber("cmd_vel_joy", Twist, callback_joy)
rospy.Subscriber("cmd_vel_auto", Twist, callback_auto)
rospy.Subscriber("cmd_vel_pedal", Twist, callback_pedal)
#rospy.Subscriber("dist_coeff", Float32, callback_lidar)
pub_udpcmd = rospy.Publisher('cmd_vel', UDPmessage, queue_size=5)
pub_cmdvel = rospy.Publisher('cmd_vel_twist', Twist, queue_size=5)
stringpub = rospy.Publisher('state_string', String, queue_size=5)

if ((robot_model!=1) and (robot_model!=2)):
    rospy.logerr("NO VALID VALUE FOR robot_model")

print("LOG PARAMETERS: \n"
      "max_lin_vel=" , max_lin_vel , "\n"
      "min_lin_vel=" , min_lin_vel , "\n"
      "max_ang_vel=" , max_ang_vel , "\n"
      "max_lin_acc=" , max_lin_acc , "\n"
      "max_ang_acc=" , max_ang_acc , "\n"
      "robot_model=" , robot_model , "\n"
      "vel_reduction_coeff=" , vel_reduction_coeff , "\n"
      "local_joy=" , local_joy)

######### SMACH INITIALIZATION #########

# Create a SMACH state machine
sm = smach.StateMachine(outcomes=[])

# Open the container
with sm:
    # Add states to the container
    smach.StateMachine.add('DISABLED', Disabled(), 
                           transitions={'joystick':'JOYSTICK',
                                        'pedal':'PEDAL',
                                        'auto':'AUTO'})
    smach.StateMachine.add('JOYSTICK', Joystick(), 
                           transitions={'disabled':'DISABLED', 
                                        'pedal':'PEDAL'})
    smach.StateMachine.add('PEDAL', Pedal(),
                           transitions={'disabled':'DISABLED',
                                        'auto':'AUTO'})
    smach.StateMachine.add('AUTO', Auto(), 
                           transitions={'disabled':'DISABLED', 
                                        'joystick':'JOYSTICK'})

# Execute SMACH plan
outcome = sm.execute()