#!/usr/bin/env python2
# FILE			: parameter.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, October 19 (UTC+0)
# MAINTAINER	: K.Supasan

# README
#   This file will use to initial control part about mode to start

# REFERENCE

import rospy

class ControlParameter:

    def __init__( self ):
        self.time_out = 1

        self.table_file = { "package" : 'zeabus_control' ,
                'directory' : 'parameter',
                'file' : 'throttle_force_table.txt'
        }

        self.topic_target_velocity = rospy.get_param('topic_target','/control/target_velocity' )
        self.topic_state = rospy.get_param('topic_state','/localize/zeabus' )

        self.topic_output = rospy.get_param( 'topic_output' , '/control/throttle' ) 


        self.target_frame = rospy.get_param('target_frame' , 'base_link_target')
        self.frame = rospy.get_param( 'own_frame' , 'base_link' )

        self.rate = rospy.get_param( 'frequency' , 10 )
