#!/usr/bin/env python2
# FILE			: fuzzy_velocity.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, December 14 (UTC+0)
# MAINTAINER	: K.Supasan

# README
#   This file will be interface to connect with other node
#   This node have responsible to navigate wii send 

# REFERENCE

class TABLEZ( object ):
    ERROR_POSITION = ( -2.5 , -1.0 , 0.0 , 0.1 , 1 , 2.5 )
    CURRENT_VELOCITY = ( -0.5 , -0.1 , 0 , +0.1 , +0.5 )
    ACCELERATION = ( -0.1 , -0.05 , 0 , +0.05 , +0.1 )

class TABLEROLL( object ):
    ERROR_POSITION = ( -1 , -0.5 , 0 , 0.5 , 1 )
    CURRENT_VELOCITY = ( -0.5 , -0.25 , 0 , 0.25 , 0.5 )
    ACCELERATION = ( -0.25 , -0.1 , 0 , 0.1 , 0.25 )

class TABLEPITCH( object ):
    ERROR_POSITION = ( -1 , -0.5 , 0 , 0.5 , 1 )
    CURRENT_VELOCITY = ( -0.5 , -0.25 , 0 , 0.25 , 0.5 )
    ACCELERATION = ( -0.25 , -0.1 , 0 , 0.1 , 0.25 )

class TABLEYAW( object ):
    ERROR_POSITION = ( -1 , -0.5 , 0 , 0.5 , 1 )
    CURRENT_VELOCITY = ( -0.5 , -0.25 , 0 , 0.25 , 0.5 )
    ACCELERATION = ( -0.35 , -0.2 , 0 , 0.2 , 0.35 )
     
class FuzzyVelocity:
    
    def __init__( self , mode ):
        if mode == 'z':
            self.set_error_position( TABLEZ.ERROR_POSITION )
            self.set_current_velocity( TABLEZ.CURRENT_VELOCITY )
            self.set_acceleration( TABLEZ.ACCELERATION )
        elif mode == 'roll' :
            self.set_error_position( TABLEROLL.ERROR_POSITION )
            self.set_current_velocity( TABLEROLL.CURRENT_VELOCITY )
            self.set_acceleration( TABLEROLL.ACCELERATION )
        elif mode == 'pitch' :
            self.set_error_position( TABLEPITCH.ERROR_POSITION )
            self.set_current_velocity( TABLEPITCH.CURRENT_VELOCITY )
            self.set_acceleration( TABLEPITCH.ACCELERATION )
        elif mode == "yaw" :
            self.set_error_position( TABLEYAW.ERROR_POSITION )
            self.set_current_velocity( TABLEYAW.CURRENT_VELOCITY )
            self.set_acceleration( TABLEYAW.ACCELERATION )
        else:
            None

        self.reset_summation()

    def reset_summation( self ):
        self.target_velocity = 0

    def set_error_position( self , table ):
        self.table_error = table
        
    def set_current_velocity( self , table ):
        self.table_velocity = table

    def set_acceleration( self , table ):
        self.table_acceleration = table

    def calculator( self , error , current_velocity ):
        fuzzy_error = 0
        for run in range( 0 , 4 ):
            if run == 0 :
                if error <= self.table_error[ run ]:
                    fuzzy_error = run
                    break
                if 
            elif run == 3 :
                if error >= self.table_error[ run ]:
                    fuzzy_error = run
                    break
            else:
                if error > self.table_error[ run - 1 ] and error <= self.table_error[ run ] :
                    fuzzy_error = run
                    break
        fuzzy_velocity = 0
        for run in range( 0 , 4 ):
            if run == 0 :
                if error <= self.table_velocity[ run ]:
                    fuzzy_velocity = run
                    break
                if 
            elif run == 3 :
                if error >= self.table_velocity[ run ]:
                    fuzzy_velocity = run
                    break
            else:
                if error > self.table_velocity[ run - 1 ] and error <= self.table_velocity[ run ] :
                    fuzzy_velocity = run
                    break
