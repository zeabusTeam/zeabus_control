#!/usr/bin/env python2
# FILE			: step_velocity.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, December 14 (UTC+0)
# MAINTAINER	: K.Supasan

# README
#   This file will be interface to connect with other node
#   This node have responsible to navigate wii send 

# REFERENCE

class TABLEX( object ):
    TABLE_ERROR = (  -2 , -1 , -0.1 , 0.1 , 1 , 2  )
    TABLE_TARGET_VELOCITY = ( -1 , -0.5 , -0.2 , 0 , 0.2 , 0.5 , 1 )

class TABLEY( object ):
    TABLE_ERROR = (  -2 , -1 , -0.1 , 0.1 , 1 , 2  )
    TABLE_TARGET_VELOCITY = ( -1 , -0.5 , -0.2 , 0 , 0.2 , 0.5 , 1 )

class TABLEZ( object ):
    TABLE_ERROR = (  -1 , -0.5 , -0.1 , 0.1 , 0.5 , 1  )
    TABLE_TARGET_VELOCITY = ( -0.5 , -0.25 , -0.1 , 0 , 0.1 , 0.25 , 0.5 )

class TABLEROLL( object ):
    TABLE_ERROR = ( -0.5 , -0.1 , -0.05 , 0.05 , 0.1 , 0.5 )
    TABLE_TARGET_VELOCITY = ( -0.6 , -0.3 , -0.1 , 0 , 0.1 , 0.3 , 0.6 )

class TABLEPITCH( object ):
    TABLE_ERROR = ( -0.5 , -0.1 , -0.05 , 0.05 , 0.1 , 0.5 )
    TABLE_TARGET_VELOCITY = ( -0.6 , -0.3 , -0.1 , 0 , 0.1 , 0.3 , 0.6 )

class TABLEYAW( object ):
    TABLE_ERROR = ( -0.5 , -0.1 , -0.02 , 0.02 , 0.1 , 0.5 )
    TABLE_TARGET_VELOCITY = ( -0.6 , -0.3 , -0.1 , 0 , 0.1 , 0.3 , 0.6 )

class StepVelocity:
    
    def __init__( self , table , max_step = 0.05 ):
        self.set_table_error( table.TABLE_ERROR )
        self.set_table_velocity( table.TABLE_TARGET_VELOCITY )
        self.reset_state()

    def reset_state( self ):
        self.save_velocity = 0

    def calculate( self , error ):
        answer = 0
        limit_order = len( self.table_error )
        for run in range( 0 , limit_order ):
            if run == 0:
                if error < self.table_error[ run ]:
                    answer = self.table_velocity[ run ]
            else:
                if error >= self.table_error[ run - 1 ] and error < self.table_error[ run ]:
                    answer = self.table_velocity[ run ]
                elif error > self.table_error[ run ]:
                    answer = self.table_velocity[ run + 1 ]
                else:
                    None
        return answer

    def set_table_error( self , table ):
        self.table_error = table

    def set_table_velocity( self , table ):
        self.table_velocity = table
 

