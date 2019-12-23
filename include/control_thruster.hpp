// FILE			: control_thruster.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, December 23 
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <mutex>

#include    <vector>

#include    <iostream>

#include    <ros/ros.h>

#include    <zeabus/escape_code.hpp>

#include    <zeabus_utility/Float64Array8.h>

#include    <zeabus_utility/Int16Array8.h>

#include    <zeabus_elec_ros/MessageTelemetryValue.h>

#include    <pid.hpp> // include form package zeabus_control

#include    <control_thruster_reconfigure.hpp>

#include    <zeabus/ros/dynamic_reconfigure.hpp>

#include    <zeabus/ros/subscriber/base_class.hpp>

#include    <zeabus/ros/node.hpp>

#include    <zeabus_utility/SendThrottle.h>

extern double parameter[4];
extern bool avaliable_new_parameter;
extern std::mutex lock_dynamic_reconfigure;
