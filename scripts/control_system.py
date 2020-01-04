#!/usr/bin/env python2
# FILE			: control_system.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, December 14 (UTC+0)
# MAINTAINER	: K.Supasan

# README
#   This file will be interface to connect with other node

# REFERENCE

# PARAMETER
__PRINT_REPORTER__ = True

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

from zeabus_utility.msg import Int16Array8, ControlCommand, Float64Array8 , Float64Array
from zeabus_utility.srv import SendBool, SendBoolResponse
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

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
        # Use save current force 
        self.lock_current_force = allocate_lock()
        self.message_current_force = nm.float64_array8()
        self.current_force = nm.float64_array8()
        # Use save addition force // frame base link
        self.lock_addition_force = allocate_lock()
        self.message_addidition_force = Float64Array()
        self.addition_force = Float64Array()
        # use Publish data to control_thruster node
        self.message_command = Float64Array8()
        self.message_command.header.frame_id = "base_link"
        # PID varialbe
        self.system = { "x" : PID() , "y" : PID() , "z" : PID() , 
                "roll" : PID() , "pitch" : PID() , "yaw" : PID() }
        self.tuning = TuneParameter( self.system, pm._PACKAGE, pm._DIRECTORY, pm._FILE_TUNE )

        self.time_stamp = rospy.get_rostime() # Update this before use load function

        self.target_force_odom_frame = [ 0 , 0 , 0 , 0 , 0 , 0 ]
        self.target_force_robot_frame = [ 0 , 0 , 0 , 0 , 0 , 0 ]

        self.saturation = [0 , 0 , 0 , 0 , 0 , 0 ]
        
        self.sum_force = np.zeros( 6 )

        self.activate_state = True # Start by activate

#   part manage variable in ros system
        self.subscribe_odom_error = rospy.Subscriber( 
            pm._TOPIC_INPUT_ERROR, 
            ControlCommand, 
            self.callback_odom_error )
    
        self.subscribe_state = rospy.Subscriber(
            pm._TOPIC_INPUT_STATE ,
            Odometry,
            self.callback_state )

        self.subscribe_current_force = rospy.Subscriber(
            pm._TOPIC_INPUT_CURRENT_FORCE,
            Float64Array8,
            self.callback_current_force )

        self.subscribe_addition_force = rospy.Subscriber(
            pm._TOPIC_INPUT_ADDITION_FORCE,
            Float64Array,
            self.callback_addition_force )

        self.publish_target_force = rospy.Publisher(
            pm._TOPIC_OUTPUT_TARGET_FORCE,
            Float64Array8,
            queue_size = 1 )

        self.server_activate_control = rospy.Service(
            pm._TOPIC_INPUT_ACTIVATE,
            SendBool,
            self.callback_activate )

#   Finish function constuctor and start function activate

    def activate( self ):
        rate = rospy.Rate( pm._RATE )

        while not rospy.is_shutdown():
            rate.sleep()
            self.time_stamp = rospy.get_rostime() # get current time

            if not self.activate_state:
                for key, run in _PARING_ORDER:
                    self.system[ key ].reset()
                continue

            # load data from subscribe
            self.load_odom_error()
            self.load_state()

#   Part find saturation of robot 
            if self.load_current_force() :
                real_force = np.zeros( 8 )
                for run in range( 0 , 8 ):
                    real_force[ run ] = self.current_force.data[ run ]
                 
                real_force = np.matmul( real_force , robot.direction )
                # Mapped robot frame to odom frame
                # We must to delete addtion force to ensure remain only value form pid + offset
                real_force[ 0 ] , real_force[ 1 ] , real_force[ 2 ] = ( 
                        self.robot_orientation.rotation( ( 
                                real_force[0] - self.addition_force.data[ 0 ] , 
                                real_force[1] - self.addition_force.data[ 1 ] , 
                                real_force[2] - self.addition_force.data[ 2 ] , 
                                0 ) )[ 0 : 3 ]
                )

                for run in range( 0 , 6 ):
                    self.saturation[ run ] = real_force[ run ]
            else:
                for run in range( 0 , 6 ):
                    self.saturation[ run ] = None 

#   Put data to PID system
            for key , run in _PARING_ORDER :
                if self.odom_error.mask[run ]:
                    self.target_force_odom_frame[ run ] =  self.system[ key ].calculate( 
                            self.odom_error.target[ run ], 
                            self.saturation[ run ] )
                else:
                    self.target_force_odom_frame[ run ] = 0

            self.calculate_force_thruster()

#            self.publish_command_thruster.publish( self.message_command )
            self.publish_target_force.publish( self.message_command )

            if __PRINT_REPORTER__ :
                self.__report()
            else:
                None

        self.tuning.save_parameter()

        rospy.signal_shutdown( "node control_system end program")

    def calculate_force_thruster( self ):
        
        robot_linear_force = self.robot_orientation.inverse_rotation( ( 
                self.target_force_odom_frame[ 0 ], 
                self.target_force_odom_frame[ 1 ], 
                self.target_force_odom_frame[ 2 ], 
                0 ) )

        self.load_addition_force()

        self.target_force_robot_frame[ 0 ] = ( robot_linear_force.vector[0] + 
                self.addition_force.data[ 0 ] )
        self.target_force_robot_frame[ 1 ] = ( robot_linear_force.vector[1] +
                self.addition_force.data[ 1 ] )
        self.target_force_robot_frame[ 2 ] = ( robot_linear_force.vector[2] +
                self.addition_force.data[ 2 ] )
        self.target_force_robot_frame[ 3 ] = ( self.target_force_odom_frame[ 3 ] +
                self.addition_force.data[ 3 ] )
        self.target_force_robot_frame[ 4 ] = ( self.target_force_odom_frame[ 4 ] +
                self.addition_force.data[ 4 ] )
        self.target_force_robot_frame[ 5 ] = ( self.target_force_odom_frame[ 5 ] +
                self.addition_force.data[ 5 ] )

        self.sum_force = np.array( self.target_force_robot_frame )

        thruster_force = np.matmul( robot.direction_inverse.T , self.sum_force.T )
        # Now we get about force of individual thruster

        self.message_command.header.stamp = self.time_stamp
        self.message_command.data = tuple( thruster_force )
                
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
        print( "add force   :{:6.2f}{:6.2f}{:6.2f}{:6.2f}{:6.2f}{:6.2f}".format(
                self.addition_force.data[0] , self.addition_force.data[1] , 
                self.addition_force.data[2] , self.addition_force.data[3] , 
                self.addition_force.data[4] , self.addition_force.data[5] ) )
        print( "robot force :{:6.2f}{:6.2f}{:6.2f}{:6.2f}{:6.2f}{:6.2f}".format(
                self.target_force_robot_frame[0] , self.target_force_robot_frame[1] , 
                self.target_force_robot_frame[2] , self.target_force_robot_frame[3] , 
                self.target_force_robot_frame[4] , self.target_force_robot_frame[5] ) )
        if self.saturation[ 0 ] != None :
            print( "real force  :{:6.2f}{:6.2f}{:6.2f}{:6.2f}{:6.2f}{:6.2f}".format(
                    self.saturation[0] , self.saturation[1] , self.saturation[2] , 
                    self.saturation[3] , self.saturation[4] , self.saturation[5] ) )
        else:
            print( "real force  : None Avaliable")
        print( "command     :{:8.3f}{:8.3f}{:8.3f}{:8.3f}{:8.3f}{:8.3f}{:8.3f}{:8.3f}".format(
                self.message_command.data[0] , self.message_command.data[1] ,
                self.message_command.data[2] , self.message_command.data[3] ,
                self.message_command.data[4] , self.message_command.data[5] ,
                self.message_command.data[6] , self.message_command.data[7] ) )
        print( "current     :{:8.3f}{:8.3f}{:8.3f}{:8.3f}{:8.3f}{:8.3f}{:8.3f}{:8.3f}\n".format(
                self.current_force.data[0] , self.current_force.data[1] ,
                self.current_force.data[2] , self.current_force.data[3] ,
                self.current_force.data[4] , self.current_force.data[5] ,
                self.current_force.data[6] , self.current_force.data[7] ) )

#   End part function for activate and start part callback
    def callback_current_force( self , message ):
        with self.lock_current_force:
            self.message_current_force = message
            self.message_current_force.header.stamp = rospy.get_rostime()

    def load_current_force( self ):
        with self.lock_current_force:
            if self.current_force.header.stamp != self.message_current_force.header.stamp:
                self.current_force = self.message_current_force

        if( self.time_stamp - self.current_force.header.stamp ).to_sec() < pm._TIMEOUT:
            activate = True
        else:
            activate = False

        return activate

    def callback_odom_error( self , message ):
        with self.lock_odom_error:
            self.message_odom_error = message
            self.message_odom_error.header.stamp = rospy.get_rostime()

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

        self.robot_orientation = Quaternion( ( self.state.pose.pose.orientation.x,
                self.state.pose.pose.orientation.y,
                self.state.pose.pose.orientation.z,
                self.state.pose.pose.orientation.w ) )

        if( self.time_stamp - self.state.header.stamp ).to_sec() < pm._TIMEOUT :
            activate = True 
        else:
            activate = False

        return activate

    def load_addition_force( self ):
        with self.lock_addition_force:
            self.addition_force = self.message_addidition_force

        if( self.time_stamp - self.state.header.stamp ).to_sec() < pm._TIMEOUT :
            activate = True 
        else:
            activate = False

        return activate

    def callback_addition_force( self , message ):
        with self.lock_addition_force:
            self.message_addidition_force = message
            self.message_addidition_force.header.stamp = rospy.get_rostime()

    def callback_activate( self , request ):
        self.activate_state = request.data
        return SendBoolResponse()
#   end part of callback and load data

if __name__=='__main__' :
    control_system_node = ControlSystem()
    control_system_node.activate()
