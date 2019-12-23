// FILE			: control_thruster_reconfigure.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, December 23 
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <control_thruster_reconfigure.hpp>

double parameter[4] = { 0 , 0 , 0 , 0 };

bool avaliable_new_parameter = false;

std::mutex lock_dynamic_reconfigure;

void dynamic_reconfigure_callback( zeabus_control::PIDThrusterConfig &config , uint32_t level )
{
    std::cout   << "Dynamic reconfigure callback " << level << "\n";
    
    lock_dynamic_reconfigure.lock();
    parameter[ 0 ] = config.kp;
    parameter[ 1 ] = config.ki;
    parameter[ 2 ] = config.kd;
    parameter[ 3 ] = config.ks;
    avaliable_new_parameter = true;
    lock_dynamic_reconfigure.unlock(); 
}

void dynamic_reconfigure_set_parameter( PID* pid )
{
    lock_dynamic_reconfigure.lock();
    for( unsigned int run  = 0 ; run < 8 ; run++ )
    {
        pid[ run ].set_parameter( parameter[0] , parameter[1] , parameter[2] , parameter[3] );
    }
    lock_dynamic_reconfigure.unlock();
}
