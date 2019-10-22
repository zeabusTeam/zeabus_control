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

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from thread import allocate_lock
from zeabus.control.pid_z_transform import PIDZTransform
from zeabus.control.lookup_pwn_force import  LookupPwmForce
from zeabus.math.quaternion import Quaternion
import zeabus.ros.message
import zeabus_robot as robot
from parameter import ControlParameter

from zeabus_utility.msg import Int16Array8

class Control:

    def __init__( self ):
        # init node is ros system
        rospy.init_node( 'control' )
        # Mode to control type of control how to get target velocity
        # PID module can look on zeabus_library/python_src/zeabus/control
        self.pid = { "x" : PIDZTransform() , "y" : PIDZTransform() ,
                "z" : PIDZTransform() , "roll" : PIDZTransform() , 
                "pitch" : PIDZTransform() , "yaw" : PIDZTransform() }
        # Below variable use to collect error for input PID
        self.error = { "x" : 0.0 , "y" : 0.0 , "z" : 0.0 , 
                "roll" : 0.0 , "pitch" : 0.0 , "yaw" : 0.0 }
        # Below variable we collect answer for you
        self.target_force_odom_frame = { "x" : 0.0 , "y" : 0.0 , "z" : 0.0 , 
                "roll" : 0.0 , "pitch" : 0.0 , "yaw" : 0.0 }
        # Below variable use to collect mode if true use target velocity from subscribe topic
        #   if false you calculate target velocity from state of robot 
        #   about state of robot we will get state from tf data ( Transformation Data )
        # Object for get param in ros system to confogure control system
        self.mode = { "x" : True , "y" : True , "z" : True , 
                "roll" : True , "pitch" : True , "yaw" : True }
        self.system_param = ControlParameter()
        # look compare find thruster can look on zeabus_library/python_src/zeabus/control
        self.lookup = LookupPwmForce( self.system_param.table_file['package'] , 
                self.system_param.table_file['derectory'] , 
                self.system_param.table_file['file'] )
        # self.mode will collect you will have to use transformation to decision
        #   target velocity or listen target velocity by time out
        self.mode = False

        self.lock_current_state = thread.allocate_lock()
        self.lock_target_velocity = thread.allocate_lock()

        # Below variable use to get value from message by with operator of locker
        self.load_target_velocity = message.twist_stamped()
        self.load_current_state = message.odometry()
        # Below variable have for boundary jump value of target
        self.real_target_velocity = message.twist_stamped()
        self.save_target_velocity = message.twist_stamped()

        # Below variable is use to save message from subscribe
        self.message_target_velocity = message.twist_stamped()
        self.message_current_state = message.odometry()
        # Below variable is command to send for command thruster
        self.message_command = Int16Array8()
        self,message_command.header.frame = self.system_param.frame

        # set up ros part
        self.subscribe_target_velocity = rospy.Subscriber(
                self.system_param.topic_target_velocity,
                TwistStamped , 
                self.listen_target_velocity
        )

        self.subscribe_current_state = rospy.Subscriber(
                self.system_param.topic_state ,
                Odometry ,
                self.listen_state
        )

        self.publish_command_thruster = rospy.Publisher( self.system_param.topic_output ,
                Int16Array8 ,
                queue_size = 1 )

    def active( self ):
        rate = rospy.Rate( self.system_param.rate )
        while not rospy.is_shutdown() :
            # This line will get about target velocity from you 
            self.load_message_target_velocity() 
            # This line will get about current velocity for you
            self.load_message_current_state()
            # Below line will get error value
            self.calculate_error()

            for key in [ "x" , "y" , "z" , "roll" , "pitch" , "yaw" ]:
                self.target_force_odom_frame[ key ] = self.pid[ key ].calculate( self.error[ key ] )

            self.calculate_force_thruster()

            self.publish_command_thruster.publish( self.message_command ) 

    def listen_target_velocity( self , message ):
        with self.lock_target_velocity :
            self.message_target_velocity = message
        
    def listen_state( self , message ):
        with self.lock_current_state :
            self.message_current_state = message

    # Because we have put input PID from odom_frame x , y , z. Next I have to convert data to 
    #   base link frame only linear type.
    #   In this we don't get state of robot from tf data because we have receive value from
    #   subscribe listen state
    def calculate_force_thruster( self ):
        # set quaternion of robot
        robot_orientation = Quaternion( ( self.load_current_state.pose.pose.orientation.x,
                self.load_current_state.pose.pose.orientation.y,
                self.load_current_state.pose.pose.orientation.z,
                self.load_current_state.pose.pose.orientation.w ) )
        
        robot_linear_force = robot_orientation.inverse_rotation( ( 
                self.target_force_odom_frame[ "x" ], 
                self.target_force_odom_frame[ "y" ], 
                self.target_force_odom_frame[ "z" ], 
                0 ) )

        sum_force = numpy.array( [ robot_linear_force.vector[0] ,
                robot_linear_force.vector[1],
                robot_linear_force.vector[2],
                self.target_force_odom_frame[ "roll" ],
                self.target_force_odom_frame[ "pitch"],
                self.target_force_odom_frame[ "yaw"] ] )

        thruster_force = numpy.matmul( robot.direction_inverse.T , sum_force.T )
        # Now we get about force of ndividual thruster

        command_throttle = []
        for run in range( 0 , 8 ):
            temp = int( self.lookup.find_pwm( thruster_force[ run ] ) )
            command_throttle.append( temp ) 

        self.message_command.header.stamp = rospy.get_rostime()
        self.message_command.data = tuple( command_throttle ) 
        
    # linear and angular type will help you decision about wnat error from state or velocity
    #   this function will split into case linear/angular velocity if true use velocity
    def calculate_error( self ):
        self.real_target_velocity.twist.linear.x = ( 
                0 ,
                self.load_target_velocity.twist.x
                ) [ self.mode["x"] ]
        self.real_target_velocity.twist.linear.y = ( 
                0 ,
                self.load_target_velocity.twist.y
                ) [ self.mode["y"] ]
        self.real_target_velocity.twist.linear.z = ( 
                0 ,
                self.load_target_velocity.twist.z
                ) [ self.mode["z"] ]
        self.real_target_velocity.twist.angular.x = ( 
                0 ,
                self.load_target_velocity.angular.x
                ) [ self.mode["roll"] ]
        self.real_target_velocity.twist.angular.y = ( 
                0 ,
                self.load_target_velocity.angular.y
                ) [ self.mode["pitch"] ]
        self.real_target_velocity.twist.angular.z = ( 
                0 ,
                self.load_target_velocity.angular.z
                ) [ self.mode["yaw"] ]
        # I have to bound target velocity
        self.boundary_target_velocity()
        self.error["x"] = self.save_target_velocity.twist.linear.x -
                self.load_current_state.twist.twist.linear.x
        self.error["y"] = self.save_target_velocity.twist.linear.y -
                self.load_current_state.twist.twist.linear.y
        self.error["z"] = self.save_target_velocity.twist.linear.z -
                self.load_current_state.twist.twist.linear.z
        self.error["roll"] = self.save_target_velocity.twist.angular.x -
                self.load_current_state.twist.twist.angular.x
        self.error["pitch"] = self.save_target_velocity.twist.angular.y -
                self.load_current_state.twist.twist.angular.y
        self.error["yaw"] = self.save_target_velocity.twist.angular.z -
                self.load_current_state.twist.twist.angular.z

    def boundary_target_velocity( self ):
        self.save_target_velocity.twist.linear.x = self.get_save_target_velocity(
                self.real_target_velocity.twist.linear.x , 
                self.save_target_velocity.twist.linear.x )
        self.save_target_velocity.twist.linear.y = self.get_save_target_velocity(
                self.real_target_velocity.twist.linear.y , 
                self.save_target_velocity.twist.linear.y )
        self.save_target_velocity.twist.linear.z = self.get_save_target_velocity(
                self.real_target_velocity.twist.linear.z , 
                self.save_target_velocity.twist.linear.z )
        self.save_target_velocity.twist.angular.x = self.get_save_target_velocity(
                self.real_target_velocity.twist.angular.x , 
                self.save_target_velocity.twist.angular.x )
        self.save_target_velocity.twist.angular.y = self.get_save_target_velocity(
                self.real_target_velocity.twist.angular.y , 
                self.save_target_velocity.twist.angular.y )
        self.save_target_velocity.twist.angular.z = self.get_save_target_velocity(
                self.real_target_velocity.twist.angular.z , 
                self.save_target_velocity.twist.angular.z )

    def get_save_target_velocity( self , target , save ):
        if abs( target - save ) > 0.5 :
            save += ( -0.5 , 0.5 ) [ target > 0 ]
        else:
            save = target
        return save

    def load_message_target_velocity( self ):
        with self.lock_target_velocity:
            self.load_target_velocity = self.message_target_velocity

    def load_message_current_state( self ):
        with self.lock_current_state:
            self.load_current_state =  self.,message_current_state

if __name__=='__main__':
    control_node = Control()
    control_node.active()
