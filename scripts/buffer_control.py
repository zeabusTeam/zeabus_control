#!/usr/bin/env python2
# FILE			: buffer_control.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, October 24 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

import rospy
import math
import numpy as np

from geometry_msgs.msg import TwistStamped

from zeabus_utility.srv import SendThrottle
from zeabus_utility.msg import Int16Array8

from thread import allocate_lock
from zeabus.control.lookup_pwm_force import  LookupPwmForce
from parameter import ControlParameter
import zeabus_robot as robot
from zeabus.ros import message as new_message

class BufferControl:

    def __init__( self ):
    
        rospy.init_node( 'control_thruster' )

        self.time_out = 0.5
        mode = True

        self.twist_message = new_message.twist_stamped()
        self.throttle_message = new_message.int16_array8()

        self.twist_load = new_message.twist_stamped()
        self.throttle_load = new_message.int16_array8()

        self.lock_twist_message = allocate_lock()
        self.lock_throttle_message = allocate_lock()

        self.subscribe_throttle_message = rospy.Subscriber(
                "/control/throttle",
                Int16Array8 , 
                self.listen_throttle_message
        )

        self.subscribe_twist_message = rospy.Subscriber(
                "/control/twist" ,
                TwistStamped ,
                self.listen_twist_message
        )

        self.publish_throttle = rospy.Publisher( "/hardware/thruster_throttle" ,
                Int16Array8 ,
                queue_size = 1 )

        self.system_param = ControlParameter()
        # look compare find thruster can look on zeabus_library/python_src/zeabus/control
        self.lookup = LookupPwmForce( self.system_param.table_file['package'] , 
                self.system_param.table_file['directory'] , 
                self.system_param.table_file['file'] )

        self.client_throttle = rospy.ServiceProxy(
                '/hardware/thruster_throttle' , SendThrottle )

    def active( self ):
        rate = rospy.Rate( 30 )
        buffer_throttle_data = ( 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 )
        active_throttle_data = buffer_throttle_data
        print( "Output Data : " + repr( buffer_throttle_data) )
        header = new_message.header( "base_link")
        message = new_message.int16_array8( buffer_throttle_data )
        while not rospy.is_shutdown():

            if self.load_twist_message():
                temp_array = np.array( [ self.twist_load.twist.linear.x ,
                        self.twist_load.twist.linear.y,
                        self.twist_load.twist.linear.z,
                        self.twist_load.twist.angular.x,
                        self.twist_load.twist.angular.y,
                        self.twist_load.twist.angular.z
                ] )
                thruster_force = np.matmul( robot.direction_inverse.T , temp_array.T )
                throttle = []
                for run in range( 0 , 8 ):
                    temp = int( self.lookup.find_pwm( thruster_force[ run ] ) )
                    throttle.append( temp )
                active_throttle_data = tuple( throttle ) 
            elif self.load_throttle_message():
                active_throttle_data = self.throttle_load.data
            else:
                active_throttle_data = ( 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 )

            if active_throttle_data != buffer_throttle_data :
                print( "Output Data : " + repr( active_throttle_data ) )
                buffer_throttle_data = active_throttle_data
            else:
                None
            header.stamp = rospy.get_rostime()

            try:
                self.client_throttle( header , buffer_throttle_data )
                message.header = header
                message.data = buffer_throttle_data 
                self.publish_throttle.publish( message )
            except rospy.ServiceException , e : 
                rospy.logfatal( "Failure to write pwm response from haredware")

            rate.sleep()
                
    def load_twist_message( self ):
        with self.lock_twist_message :
            self.twist_load = self.twist_message
        active = False
        if ( rospy.get_rostime() - self.twist_load.header.stamp ).to_sec() < self.time_out :
            active = True
        return active

    def load_throttle_message( self ):
        with self.lock_throttle_message : 
            self.throttle_load = self.throttle_message
        active = False
        if ( rospy.get_rostime() - self.throttle_load.header.stamp ).to_sec() < self.time_out :
            active = True
        return active

    def listen_twist_message( self , message ):
        with self.lock_twist_message :
            self.twist_message = message
            self.twist_message.header.stamp = rospy.get_rostime()

    def listen_throttle_message( self , message ):
        with self.lock_throttle_message :
            self.throttle_message = message
            self.throttle_message.header.stamp = rospy.get_rostime()

if __name__=='__main__':
    buffer_node = BufferControl()
    buffer_node.active()
