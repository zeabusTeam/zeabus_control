#!/usr/bin/env python2
# FILE			: control.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, October 10 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

import rospy
import math
import numpy as np
import zeabus.control.zeabus_robot as robot

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from thread import allocate_lock
from zeabus.control.pid_z_transform import PIDZTransform
from zeabus.control.lookup_pwn_force import  LookupPwmForce
from parameter import ControlParameter

class Control:

    def __init__( self ):
        # init node is ros system
        rospy.init_node( 'control' )
        # Mode to control type of control how to get target velocity
        self.mode = True # if true will receive target velocity by subscbribe
        # PID module can look on zeabus_library/python_src/zeabus/control
        self.pid = PIDZTransform()
        # Object for get param in ros system to confogure control system
        self.system_param = ControlParameter()
        # look compare find thruster can look on zeabus_library/python_src/zeabus/control
        self.lookup = LookupPwmForce( self.system_param.table_file['package'] , 
                self.system_param.table_file['derectory'] , 
                self.system_param.table_file['file'] )
        # self.mode will collect you will have to use transformation to decision
        #   target velocity or listen target velocity by time out
        self.mode = False

        self.current_state_locker = thread.allocate_lock()
        self.target_velocity_locker = thread.allocate_lock()

        self.load_target_velocity = TwistStamped()
        self.load_current_state = Odometry()
        # Below variable have for boundary jump value of target
        self.save_target_velocity = TwistStamped()

        self.message_target_velocity = TwistStamped()
        self.message_current_state = Odometry()

        # set up ros part
        self.subscribe_target_velocity = rospy.Subscriber(
            self.system_param.topic_target_velocity,
            TwistStamped , 
            self.listen_target_velocity
        )

        self.current_state = rospy.Subscriber(
            self.system_param.topic_state ,
            Odometry ,
            self.listen_state
        )
        
    def active( self ):
        # This mode if we will want to calcuate target velocity by own self from position
        if( mode ):
            self.load_message_current_state()
        else:
            self.load_message_target_velocity()

    def listen_target_velocity( self , message ):
        None
        
    def listen_state( self , message ):
        None

    def load_message_target_velocity( self ):
        with self.target_velocity_locker:
            self.load_target_velocity = self.message_target_velocity

    def load_message_current_state( self ):
        with self.current_velocity_locker:
            self.load_current_state =  self.,message_current_state

if __name__=='__main__':
    control_node = Control()
    control_node.active()
