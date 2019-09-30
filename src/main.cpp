// FILE			: main.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, September 30 (UTC+0)
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README
//  Boost on I work is version 1.69.1

// REFERENCE
//  Thruster mapper equation is sum_force_&_torque = matrix_characteristic * thruster_force
//      Dimension is ( 1 , 6 ) = ( 1 , 8 ) * (  8 * 6 )
//      have inverse to ( 1 , 6 ) * ( 6 , 8 ) = ( 1 , 8 )

// MACRO SET
#define thruster_number 8

// MACRO CONDITION

#include    <cmath>

#include    <iostream>

#include    <boost/qvm/mat_operations.hpp>

#include    <boost/numeric/ublas/matrix_proxy.hpp>

#include    <boost/numeric/ublas/matrix.hpp>

#include    <boost/numeric/ublas/io.hpp>

#include    <zeabus/math/parameter.hpp>

namespace _ublas = boost::numeric::ublas;

#include    "test.cpp"

#include    "thruster.cpp"

_ublas::matrix< double > inverse_characteristic( thruster_number , 6 );
_ublas::matrix< double > characteristic( 6 , thruster_number );
_ublas::matrix< double > target_summation( 6 , 1 );
_ublas::matrix< double > thruster_direction( thruster_number , 3 );
_ublas::matrix< double > thruster_distance( thruster_number , 3 );
_ublas::matrix< double > thruster_arm( thruster_number , 3 );

int main( int argv , char** argc )
{
    set_default();
    check_matrix();
} 
