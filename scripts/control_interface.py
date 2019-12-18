#!/usr/bin/env python2
# FILE			: control_interface.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, December 14 (UTC+0)
# MAINTAINER	: K.Supasan

# README
#   This file will be interface to connect with other node
#   This node have responsible to navigate wii send 

# REFERENCE

import rospy
import math
import numpy as np # numpy
import tf

from thread import allocate_lock
from zeabus.math.quaternion import Quaternion
from zeabus.ros import message as nm # new_message

import step_velocity as ctv # control velocity
from constant import Interface as pm # parameter
from constant import _PARING_ORDER

from nav_msgs.msg import Odometry
from zeabus_utility.msg import ControlCommand

class ControlInterface :

    def __init__( self ):
        rospy.init_node( 'control_interface' )

#   Section manage variable
        # Use save currunt state
        self.lock_state = allocate_lock()
        self.message_state = nm.odometry()
        self.current_velocity = [ 0 , 0 , 0 , 0 , 0 , 0 ]
        self.current_quaternion = Quaternion( ( 0 , 0 , 0 , 1 ) )
        self.time_stamp_state = rospy.get_rostime()
        # Use save target velocity // FRAME base_link
        self.lock_target_velocity = allocate_lock()
        self.message_target_velocity = nm.control_command()
        self.target_velocity = nm.control_command()
        self.time_stamp_target_velocity = rospy.get_rostime()
        # Use save target velocity // FRAME odom
        self.odom_target_velocity = nm.control_command()
        # Use time stamp before load
        self.time_stamp = rospy.get_rostime()

        self.error_state = [ 0 , 0 , 0 , 0 , 0 , 0 ]

        self.output_odom_error = nm.control_command()
    
        self.listener_tf = tf.TransformListener()

        self.control_velocity = ( ctv.StepVelocity( ctv.TABLEX ), ctv.StepVelocity( ctv.TABLEY ),
                ctv.StepVelocity( ctv.TABLEZ ) , ctv.StepVelocity( cty.TABLEROLL ),
                ctv.StepVelocity( ctv.TABLEPITCH ) , ctv.StepVelocity( ctv.TABLEYAW ) )

        for run in range( 0 , 6 ):
            self.control_velocity[ run ].reset_state()

#   Section manage ros system
        self.publish_odom_error = rospy.Publisher( 
                pm._TOPIC_OUTPUT_ERROR ,
                ControlCommand,
                queue_size = 1 )

        self.subscribe_current_state = rospy.Subscriber(
                pm._TOPIC_INPUT_STATE,
                Odometry,
                self.callback_state
        )

#   end part constructor ControlInterface

    def activate( self ):
        rate = rospy.Rate( 30 )
        self.output_odom_error.header.frame_id = "odom"
        self.output_odom_error.mask = ( False , False , True , True , True , True )

        self.odom_target_velocity.header.frame_id = "odom"
        self.odom_target_velocity.mask = ( False , False , True , True , True , True )
        while not rospy.is_shutdown() :

            rate.sleep()

            self.get_error_position()

            temp = []
            for run in range( 0 , 6 ):
                temp.append( self.control_velocity[ run ].calculate( self.error_state[ run ] ) )
        
            self.odom_target_velocity.target = tuple( temp )

            self.time_stamp = rospy.get_rostime()

            temp = []
 
            if self.load_current_velocity():
                for run in range( 0 , 6 ):
                    temp.append( self.odom_target_velocity.target[ run ] - 
                            self.current_velocity[ run ] )
            else:
                temp = (0, 0, 0, 0, 0, 0 )
                print( "Warning Faliure to ger current velocity")
                
            self.output_odom_error.header.stamp = self.time_stamp
            self.output_odom_error.target = tuple( temp )
            self.publish_odom_error( self.output_odom_error )

#   end part activate function and start part update error position 

    def load_current_velocity( self ):
        twist_quaternion = Quaternion( ( 0 , 0 , 0 , 1 ) )
        activate = False
        with self.lock_state:
            if self.message_state.header.stamp != self.time_stamp_state :
                self.time_stamp_state = self.message_state.header.stamp 
                twist_quaternion.set_quaternion( ( self.message_state.twist.twist.linear.x,
                        self.message_state.twist.twist.linear.y,
                        self.message_state.twist.twist.linear.z,
                        0 ) )
                self.current_velocity[3] = self.message_state.twist.twist.angular.x
                self.current_velocity[4] = self.message_state.twist.twist.angular.y
                self.current_velocity[5] = self.message_state.twist.twist.angular.z
            else:
                activate = True

        if ( self.time_stamp - self.time_stamp_state ).to_sec() < pm._TIMEOUT and not activate:
            temp = self.current_quaternion.inverse_rotation( twist_quaternion )
            self.current_velocity[ 0 ] = temp.vector[ 0 ]
            self.current_velocity[ 1 ] = temp.vector[ 1 ]
            self.current_velocity[ 2 ] = temp.vector[ 2 ]
            activate = True
        return activate

    def get_error_position( self ):
        try:
            ( translation , rotation ) = self.listener_tf.lookupTransform( 
                    self.system_param.frame,
                    self.system_param.target_frame,
                    rospy.Time(0) )
        except( tf.LookupException , tf.ConnectivityException, tf.ExtrapolationException ):
            translation = ( 0 , 0 , 0 )
            rotation = ( 0 , 0 , 0 , 1 )

        self.error_state[ 0 ] = translation[0]
        self.error_state[ 1 ] = translation[1]
        self.error_state[ 2 ] = translation[2]
        temp_euler = Quaternion( rotation ).get_euler()
        self.error_state[ 3 ] = temp_euler[2]
        self.error_state[ 4 ] = temp_euler[1]
        self.error_state[ 5 ] = temp_euler[0]

#   end part of load data and start part callback
    
    def callback_state( self , message ):
        with self.lock_state:
            self.message_state = message
        self.current_quaternion.set_quaternion(
                self.message_state.pose.pose.orientation.x,
                self.message_state.pose.pose.orientation.y,
                self.message_state.pose.pose.orientation.z,
                self.message_state.pose.pose.orientation.w
        )

#   End part callback function

if __name__=='__main___':
    control_interface_node = ControlInterface()
    control_interface_node.activate()
