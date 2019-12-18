#!/usr/bin/env python2
# FILE			: control_interface.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, December 14 (UTC+0)
# MAINTAINER	: K.Supasan

# README
#   This file will be interface to connect with other node

# REFERENCE

import rospy
import math
import numpy as np
import tf

import zeabus_robot as robot

from thread import allocate_lock
from zeabus.ros import message as nm # new message

#from zeabus.control.pid_z_transform import PIDZTransform as PID
from zeabus.control.pid import PID as PID
from zeabus.control.lookup_pwm_force import LookupPwmForce
from zeabus.math.quaternion import Quaternion

from tune_parameter import TuneParameter
from constant import System as pm # parameter
from constant import _PARING_ORDER 

from zeabus_utility.msg import Int16Array8, ControlCommand
from nav_msgs.msg import Odometry

class ControlSystem :

    def __init__( self ):
        rospy.init_node( 'control_system' )

#   Section manage variable
        # Use save current state // use orientation for calculate force
        self.lock_state = allocate_lock()
        self.message_state = nm.odometry()
        self.state = nm.odometry()
        self.time_stamp_state = rospy.get_rostime()
        # Use save error from control_interface // frame odom
        self.lock_odom_error = allocate_lock()
        self.message_odom_error = nm.control_command()
        self.odom_error = nm.control_command()
        # use Publish data to control_thruster node
        self.message_command = Int16Array8()
        self.message_command,header.frame_id = "base_link"
        # PID varialbe
        self.system = { "x" : PID() , "y" : PID() , "z" : PID() , 
                "roll" : PID() , "pitch" : PID() , "yaw" : PID() }
        self.tuning = TuneParameter( self.system, pm._PACKAGE, pm._DIRECTORY, pm._FILE_TUNE )

        self.time_stamp = rospy.get_rostime() # Update this before use load function

        self.lookup = LookupPwmForce( pm._PACKAGE , pm._DIRECTORY , pm._FILE_TABLE )

        self.target_force_odom_frame = [ 0 , 0 , 0 , 0 , 0 , 0 ]

        self.saturation = [0 , 0 , 0 , 0 , 0 , 0 ]

#   part manage variable in ros system
        self.subscribe_odom_error = rospy.Subscriber( 
            pm._TOPIC_INPUT_ERROR, 
            ControlCommand, 
            self.callback_odom_error )
    
        self.subscribe_state = rospy.Subscriber(
            pm._TOPIC_INPUT_STATE ,
            Odometry,
            self.callback_state )

        self.publish_command_thruster = rospy.Publisher( 
            pm._TOPIC_OUTPUT_COMMAND_THROTTLE,
            Int16Array8,
            queue_size = 1 )
#   Finish function constuctor and start function activate

    def activate( self ):
        rate = rospy.Rate( pm._RATE )

        while not rospy.is_shutdown():
            rate.sleep()
            self.time_stamp = rospy.get_rostime() # get current time

            # load data from subscribe
            self.load_odom_error()
            self.load_state()

            for key , run in _PARING_ORDER :
                if self.odom_error.mask[run ] :
                    self.target_force_odom_frame[ run ] =  self.system[ key ].calculate( 
                            self.odom_error.target[ run ], 
                            self.saturation[ run ] ) )
                else:
                    self.target_force_odom_frame[ run ] = 0

            self.calculate_force_thruster()

            self.publish_command_thruster.publish( self.message_command )

            self.__report()

        self.tuning.save_parameter()

        rospy.signal_shutdown( "node control_system end program")

    def calculate_force_thruster( self ):
        # set quaternion of robot
        robot_orientation = Quaternion( ( self.state.pose.pose.orientation.x,
                self.state.pose.pose.orientation.y,
                self.state.pose.pose.orientation.z,
                self.state.pose.pose.orientation.w ) )
        
        robot_linear_force = robot_orientation.inverse_rotation( ( 
                self.target_force_odom_frame[ 0 ], 
                self.target_force_odom_frame[ 1 ], 
                self.target_force_odom_frame[ 2 ], 
                0 ) )

        self.target_force_robot_frame[ 0 ] = robot_linear_force.vector[0]
        self.target_force_robot_frame[ 1 ] = robot_linear_force.vector[1]
        self.target_force_robot_frame[ 2 ] = robot_linear_force.vector[2]
        self.target_force_robot_frame[ 3 ] = self.target_force_odom_frame[ 3 ]
        self.target_force_robot_frame[ 4 ] = self.target_force_odom_frame[ 4 ]
        self.target_force_robot_frame[ 5 ] = self.target_force_odom_frame[ 5 ]

        sum_force = np.array( self.target_force_robot_frame )

        thruster_force = np.matmul( robot.direction_inverse.T , sum_force.T )
        # Now we get about force of ndividual thruster

        command_throttle = []
        for run in range( 0 , 8 ):
            temp = int( self.lookup.find_pwm( thruster_force[ run ] ) )
            command_throttle.append( temp ) 

        # preapare message for publish to buffer control
        self.message_command.header.stamp = self.time_stamp
        self.message_command.data = tuple( command_throttle )

        real_force = np.zeros( 8 )
        for run in range( 0 , 8 ):
            real_force[ run ] = self.lookup.force_table[ command_throttle[ run ] + 1000 ]
        
        real_force = np.matmul( real_force , robot.direction )
        for run in range( 0 , 6 ):
            self.saturation[ run ] =  -real_force[ run ] + sum_force[ run ]

    def __report( self ):
        print( "=============== REPORTED ===============" )
        print( "input error :{:6.2f}{:6.2f}{:6.2f}{:6.2f}{:6.2f}{:6.2f}".format(
                self.odom_error.target[0] , self.odom_error.target[1] , 
                self.odom_error.target[2] , self.odom_error.target[3] , 
                self.odom_error.target[4] , self.odom_error.target[5] ) )
        print( "mask error  :{:6}{:6}{:6}{:6}{:6}{:6}".format(
                self.odom_error.mask[0] , self.odom_error.mask[1] , 
                self.odom_error.mask[2] , self.odom_error.mask[3] , 
                self.odom_error.mask[4] , self.odom_error.mask[5] ) )
        print( "odom force  :{:6.2f}{:6.2f}{:6.2f}{:6.2f}{:6.2f}{:6.2f}".format(
                self.target_force_odom_frame[0] , self.target_force_odom_frame[1] , 
                self.target_force_odom_frame[2] , self.target_force_odom_frame[3] , 
                self.target_force_odom_frame[4] , self.target_force_odom_frame[5] ) )
        print( "robot force :{:6.2f}{:6.2f}{:6.2f}{:6.2f}{:6.2f}{:6.2f}".format(
                self.target_force_robot_frame[0] , self.target_force_robot_frame[1] , 
                self.target_force_robot_frame[2] , self.target_force_robot_frame[3] , 
                self.target_force_robot_frame[4] , self.target_force_robot_frame[5] ) )
        print( "saturation  :{:6.2f}{:6.2f}{:6.2f}{:6.2f}{:6.2f}{:6.2f}".format(
                self.saturation[0] , self.saturation[1] , self.saturation[2] , 
                self.saturation[3] , self.saturation[4] , self.saturation[5] ) )
        print( "Throttle command : " + self.message_command.data + "\n" )

#   End part function for activate and start part callback
    def callback_odom_error( self , message ):
        with self.lock_odom_error:
            self.message_odom_error = message

    def load_odom_error( self ):
        with self.lock_odom_error:
            if self.odom_error.header.stamp != self.message_odom_error.header.stamp :
                self.odom_error = self.message_odom_error

        if ( self.time_stamp - self.odom_error.header.stamp ).to_sec() < pm._TIMEOUT :
            activate = True 
        else:
            activate = False
    
        return activate

    def callback_state( self , message ):
        with self.lock_state :
            self.message_state = message 
    
    def load_state( self ):
        with self.lock_state :
            if self.state.header.stamp != self.message_state.header.stamp :
                self.state = self.message_state

        if( self.time_stamp - self.state.header.stamp ).to_sec() < pm._TIMEOUT :
            activate = True 
        else:
            activate = False

        return activate
#   end part of callback and load data

if __name__=='__main__' :
    control_system_node = Control()
    control_system_node.activate()
