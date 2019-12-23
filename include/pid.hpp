// FILE			: pid.hpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, December 23 
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <iostream>

#include    <cmath>

#include    <ros/ros.h>

#ifndef _ZEABUS_CONTROL_PID_HPP__
#define _ZEABUS_CONTROL_PID_HPP__

template< typename first , typename second >
bool equal( first first_value , second second_value )
{
    bool result = false;
    if( fabs( first_value - second_value ) <= 1e-3 ) result = true;
    return result;
}

class PID
{
    public:
        PID( double kp = 0 , double ki = 0 , double kd = 0 , double ks = 0 );

        void set_parameter( double kp , double ki, double kd , double ks);

        void reset();

        double calculate( double error , ros::Time time_stamp , double saturation = 0 );

    protected:
        double kp;
        double ki;
        double kd;
        double ks;

        double error;
        ros::Time time_stamp;

        double summation;

}; // construct PID

#endif // _ZEABUS_CONTROL_PID_HPP__
