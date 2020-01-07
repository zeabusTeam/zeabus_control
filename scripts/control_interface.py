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
from zeabus_utility.msg import ControlCommand, BoolArray6
from zeabus_utility.srv import ServiceMask, ServiceMaskResponse

class ControlInterface :

    def __init__( self ):
        rospy.init_node( 'control_interface' )

#   Section manage variable
        # Use save currunt state
        self.lock_state = allocate_lock()
        self.lock_quaternion = allocate_lock()
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
        # Use for send localize reset
        self.message_reset = BoolArray6
        self.message_reset.header.frame_id = "base_link"

        self.error_state = [ 0 , 0 , 0 , 0 , 0 , 0 ]

        self.output_odom_error = nm.control_command()
    
        self.listener_tf = tf.TransformListener()

        self.control_velocity = ( ctv.StepVelocity( ctv.TABLEX ), ctv.StepVelocity( ctv.TABLEY ),
                ctv.StepVelocity( ctv.TABLEZ ) , ctv.StepVelocity( ctv.TABLEROLL ),
                ctv.StepVelocity( ctv.TABLEPITCH ) , ctv.StepVelocity( ctv.TABLEYAW ) )

        for run in range( 0 , 6 ):
            self.control_velocity[ run ].reset_state()

#   Section manage ros system
        self.publish_odom_error = rospy.Publisher( 
                pm._TOPIC_OUTPUT_ERROR ,
                ControlCommand,
                queue_size = 1 
        )

        self.publish_localize_reset = rospy.Publisher(
                pm._TOPIC_OUTPUT_RESET ,
                BoolArray6,
                queue_size = 1
        )

        self.subscriber_current_state = rospy.Subscriber(
                pm._TOPIC_INPUT_STATE,
                Odometry,
                self.callback_state
        )

        self.subscriber_target_velocity = rospy.Subscriber(
                pm._TOPIC_INPUT_TARGET_VELOCITY,
                ControlCommand,
                self.callback_target_velocity
        )

        self.server_mask_control = rospy.Service(
                pm._TOPIC_INPUT_MASK,
                ServiceMask,
                self.callback_mask
        )

#   end part constructor ControlInterface

    def activate( self ):
        rate = rospy.Rate( _pm.Rate )
        self.output_odom_error.header.frame_id = "odom"
        self.output_odom_error.mask = ( True , True , True , True , True , True )

        self.odom_target_velocity.header.frame_id = "odom"
        self.odom_target_velocity.mask = ( True , True , True , True , True , True )
        while not rospy.is_shutdown() :

            rate.sleep()

            self.get_error_position()

            with self.lock_quaternion:
                # Translation we have get is frame of robot
                # I have to transform to odom frame
                error_quaternion = Quaternion( ( self.error_state[ 0 ] ,
                        self.error_state[ 1 ] , 
                        self.error_state[ 2 ] , 
                        0 ) )
                error_quaternion = self.current_quaternion.rotation( error_quaternion )
            self.error_state[ 0 ] = error_quaternion[ 0 ]
            self.error_state[ 1 ] = error_quaternion[ 1 ]
            self.error_state[ 2 ] = error_quaternion[ 2 ]

            # Load message from /control/interface/target
            self.get_odom_target_velocity()
            # Next will prepare target velocity in body frame
            temp = []
            for run in range( 0 , 6 ):
                if self.target_velocity.mask[ run ]:
                    temp.append( self.target_velocity.target[ run ] )
                else:
                    temp.append( self.control_velocity[ run ].calculate( self.error_state[ run ] ) )
        
            self.odom_target_velocity.target = tuple( temp )

            self.time_stamp = rospy.get_rostime()

            temp = []
 
            if self.load_current_velocity():
                for run in range( 0 , 6 ):
                    if run in ( 3 , 4 , 5 ) :
                        temp.append( self.odom_target_velocity.target[ run ] - 
                                self.current_velocity[ run ] )
                    else:
                        temp.append( self.error_state[ run ] )
            else:
                temp = (0, 0, 0, 0, 0, 0 )
                print( "Warning Faliure to get current velocity")

            self.output_odom_error.header.stamp = self.time_stamp
            self.output_odom_error.target = tuple( temp )
            self.publish_odom_error.publish( self.output_odom_error )

            self.__report()

        rospy.signal_shutdown( "node control_system end program")

    def __report( self ):
        print( "========== REPORT INTERFACE ==========")
        print( "ERROR_STATE :{:6.2f}{:6.2f}{:6.2f}{:6.2f}{:6.2f}{:6.2f}".format( 
                self.error_state[0] , self.error_state[1] , self.error_state[2],
                self.error_state[3] , self.error_state[4] , self.error_state[5] ) ) 
        print( "VEL_MASK    :{:6}{:6}{:6}{:6}{:6}{:6}\n".format(
                self.target_velocity.mask[0] , self.target_velocity.mask[1] , 
                self.target_velocity.mask[2] , self.target_velocity.mask[3] , 
                self.target_velocity.mask[4] , self.target_velocity.mask[5] ) )
        print( "TARGET_VEL  :{:6.2f}{:6.2f}{:6.2f}{:6.2f}{:6.2f}{:6.2f}".format( 
                self.odom_target_velocity.target[0] , self.odom_target_velocity.target[1] , 
                self.odom_target_velocity.target[2] , self.odom_target_velocity.target[3] , 
                self.odom_target_velocity.target[4] , self.odom_target_velocity.target[5] ) ) 
        print( "CURRENT_VEL :{:6.2f}{:6.2f}{:6.2f}{:6.2f}{:6.2f}{:6.2f}".format( 
                self.current_velocity[0] , self.current_velocity[1] , self.current_velocity[2] , 
                self.current_velocity[3] , self.current_velocity[4] , self.current_velocity[5] ) ) 
        print( "ERROR_VEL   :{:6.2f}{:6.2f}{:6.2f}{:6.2f}{:6.2f}{:6.2f}".format( 
                self.output_odom_error.target[0] , self.output_odom_error.target[1] , 
                self.output_odom_error.target[2] , self.output_odom_error.target[3] , 
                self.output_odom_error.target[4] , self.output_odom_error.target[5] ) ) 
        print( "ERROR_MASK  :{:6}{:6}{:6}{:6}{:6}{:6}\n".format(
                self.output_odom_error.mask[0] , self.output_odom_error.mask[1] , 
                self.output_odom_error.mask[2] , self.output_odom_error.mask[3] , 
                self.output_odom_error.mask[4] , self.output_odom_error.mask[5] ) )

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
            with self.lock_quaternion:
                temp = self.current_quaternion.inverse_rotation( twist_quaternion.vector )
            self.current_velocity[ 0 ] = temp.vector[ 0 ]
            self.current_velocity[ 1 ] = temp.vector[ 1 ]
            self.current_velocity[ 2 ] = temp.vector[ 2 ]
            activate = True
        return activate

    def get_error_position( self ):
        try:
            ( translation , rotation ) = self.listener_tf.lookupTransform(
                    pm._FRAME_ERROR_PARENT,
                    pm._FRAME_ERROR_CHILD, 
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

    # In case have message target velocity this will get and transform to odom velocity
    # escape angular velocity is base_link frame
    def get_odom_target_velocity( self ):
        temp_quaternion = None
        with self.lock_target_velocity:
            if( self.time_stamp - self.message_target_velocity.header.stamp ).to_sec() < 0.2 :
                with self.lock_quaternion:
                    temp_quaternion = self.current_quaternion.inverse_rotation( ( 
                            self.message_target_velocity.target[0],
                            self.message_target_velocity.target[1],
                            self.message_target_velocity.target[2],
                            0 ) )
                self.target_velocity.target[ 4 : 6 ] = self.message_target_velocity.target[ 4 : 6 ]
                self.target_velocity.mask = self.message_target_velocity.mask

        # If temp_quaternion have value that mean you message not time out
        if temp_quaternion != None :
            self.target_velocity.target[ 0 : 3 ] = temp_quaternion[ 0 : 3 ]

            # prepare message for output reset
            self.message_reset.header.stamp = self.time_stamp
            self.message_reset.mask = self.target_velocity.mask 
            self.publish_localize_reset.publish( self.message_reset )

#   end part of load data and start part callback
    
    def callback_state( self , message ):
        with self.lock_state:
            self.message_state = message

        with self.lock_quaternion:
            self.current_quaternion.set_quaternion( (
                    self.message_state.pose.pose.orientation.x,
                    self.message_state.pose.pose.orientation.y,
                    self.message_state.pose.pose.orientation.z,
                    self.message_state.pose.pose.orientation.w
            ) )

    def callback_mask( self , request ):
        if request.activate_mask :
            self.output_odom_error.mask = request.target_mask
        return ServiceMaskResponse( self.output_odom_error.mask )

    def callback_target_velocity( self , message ):
        with self.lock_target_velocity:
            self.message_target_velocity = message
        

#   End part callback function

if __name__=='__main__':
    control_interface_node = ControlInterface()
    control_interface_node.activate()
