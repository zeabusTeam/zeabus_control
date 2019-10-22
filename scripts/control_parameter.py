#!/usr/bin/env python2
# FILE			: control_parameter.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, October 22 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE
# ref01 : http://wiki.ros.org/dynamic_reconfigure/Tutorials/UsingTheDynamicReconfigurePythonClient
# ref02 : http://wiki.ros.org/dynamic_reconfigure/Tutorials/HowToWriteYourFirstCfgFile
# ref03 : http://wiki.ros.org/dynamic_reconfigure/Tutorials/SettingUpDynamicReconfigureForANode%28python%29

import rospy

from zeabus_control.cfg import PidZTransformConfig
from dynamic_reconfigure.server import Server

from zeabus.control.pid_z_transform import PIDZTransform

class ControlParameter:

    def __init__( self , pid_id ):
        
        self.pid_id = pid_id

        self.server_parameter = Server( PIDZTransform , self.callback )

    def callback( self, config , level ):
        print( repr( config ) )        
