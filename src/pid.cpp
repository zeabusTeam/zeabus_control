// FILE			: pid.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, December 23 
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET

// MACRO CONDITION

#include    <pid.hpp>

template< typename first , typename second >
bool equal( first first_value , second second_value )
{
    bool result = false;
    if( fabs( first_value - second_value ) <= 1e-3 ) result = true;
    return result;
}

PID::PID( double kp , double ki , double kd , double ks )
{   
    this->set_parameter( kp , ki , kd , ks);
} // PID::PID constructor

void PID::set_parameter( double kp , double ki, double kd , double ks )
{
    bool have_reset = false;
    if( equal( this->kp , kp ) ) have_reset = true;
    else if( equal( this->ki , ki ) ) have_reset = true;
    else if( equal( this->kd , kd ) ) have_reset = true;
    else if( equal( this->ks , ks ) ) have_reset = true;
    else;

    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->reset();

} // PID::set_parameter

void PID::reset()
{
    this->time_stamp = ros::Time::now();
    this->summation = 0;
} // PID::reset

double PID::calculate( double error , ros::Time time_stamp , double saturation )
{
    double output = this->kp * error;
    double diff_time = ( time_stamp - this->time_stamp ).toSec();
    this->summation += ( ( error + this->error ) * this->ki  / 2  - 
            saturation * this->ks ) * diff_time;
    output += ( error - this->error ) / diff_time;
    return output + this->summation;
} // PID::calculate
