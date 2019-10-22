#!/usr/bin/env python2
# FILE			: tune_parameter.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, October 22 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE
# ref01 : http://wiki.ros.org/dynamic_reconfigure/Tutorials/UsingTheDynamicReconfigurePythonClient
# ref02 : http://wiki.ros.org/dynamic_reconfigure/Tutorials/HowToWriteYourFirstCfgFile
# ref03 : http://wiki.ros.org/dynamic_reconfigure/Tutorials/SettingUpDynamicReconfigureForANode%28python%29
# ref04 : https://docs.python.org/2.6/library/string.html

import rospy

from zeabus_control.cfg import PidZTransformConfig
from dynamic_reconfigure.server import Server

from zeabus.control.pid_z_transform import PIDZTransform
from zeabus.ros.yaml_handle import YamlHandle

class TuneParameter:

    def __init__( self , pid_id ):
        
        self.pid_id = pid_id
        
        self.reset_parameter( 0.1 , 5 )

        self.yaml_handle = YamlHandle( "zeabus_control" , "parameter" , "pid_value.yaml" )

        self.load_parameter()

        self.server_parameter = Server( PidZTransformConfig , self.callback )

    def reset_parameter( self , sampling_time , coefficients ):
        self.data_config = { "p_x" : 0 , "i_x" : 0 , "d_x" : 0 ,
                "p_y" : 0 , "i_y" : 0 , "d_y" : 0 ,
                "p_z" : 0 , "i_z" : 0 , "d_z" : 0 ,
                "p_roll" : 0 , "i_roll" : 0 , "d_roll" : 0 ,
                "p_pitch" : 0 , "i_pitch" : 0 , "d_pitch" : 0 ,
                "p_yaw" : 0 , "i_yaw" : 0 , "d_yaw" : 0 ,
                "sampling_time" : sampling_time ,
                "coefficients" : coefficients
        }

    def load_parameter( self ):
        if self.yaml_handle.check_file():
            self.data_config = self.yaml_handle.load_data()
        else:
            print( "{} file doesn\'t exists reset parameter".format( self.yaml_handle.fullpath ) )
            self.reset_parameter( 0.1 , 5 )

        self.set_parameter()

    def save_parameter( self ):
        self.yaml_handle.save_data( self.data_config )

    def set_parameter( self ):
        for key in [ "roll" , "pitch" , "yaw" , "x" , "y" , "z" ]:
            self.pid_id[ key ].set_parameter( self.data_config[ "p_" + key ] ,
                    self.data_config[ "i_" + key ] ,
                    self.data_config[ "d_" + key ] , 
                    self.data_config[ "sampling_time" ] ,
                    self.data_config[ "coefficients" ] )

    def callback( self, config , level ):
        print( "Data receive is ===================================================== " )
        print( config.keys() )
        if( level == 0 ):
            self.data_config = config
        self.report_configure()
        return self.data_config

    def report_configure( self ):
        print( "---------------------------------------------")
        print( "Type   |  P_value   |   I_value   |   D_value")
        for key in ( "x" , "y" , "z" , "roll" , "pitch" , "yaw" ):
            print( "{0:<7}|{1:9.2f}   |{2:10.2f}   |{3:9.2f}".format( key, 
                    self.data_config[ "p_" + key ],
                    self.data_config[ "i_" + key ],
                    self.data_config[ "d_" + key ] ) )
        print( "sampling_time is " + str( self.data_config["sampling_time"] ) )
        print( "coefficients is " + str( self.data_config["coefficients"] ) )
        print( "---------------------------------------------")
            
