// FILE			: control_thruster_reconfigure.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, December 23 
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <dynamic_reconfigure/server.h>

#include    <zeabus_control/PIDThrusterConfig.h>

#include    <mutex>

#include    <pid.hpp>

// column point to kp ki kd ks
// row point to 8 thruster

#ifndef _ZEABUS_CONTROL_CONTROL_THRUSTER_RECONFIGURE_HPP__
#define _ZEABUS_CONTROL_CONTROL_THRUSTER_RECONFIGURE_HPP__


void dynamic_reconfigure_callback( zeabus_control::PIDThrusterConfig &config , uint32_t level );

void dynamic_reconfigure_set_parameter( PID* pid );

#endif // _ZEABUS_CONTROL_CONTROL_THRUSTER_RECONFIGURE_HPP__
