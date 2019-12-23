// FILE			: control_thruster.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, December 23 
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET
#define _DYNAMIC_RECONFIGURE_

// MACRO CONDITION

#include    <control_thruster.hpp>

int main( int argv , char** argc )
{

    zeabus_ros::Node node( argv , argc , "control_thruster" );

    ros::NodeHandle ph("~"); // param handle
    ros::NodeHandle nh(""); // node handle

    PID pid[8];

    node.spin();

// =========================== PATH parameter =================================
    unsigned int frequency = 30;

#ifdef _DYNAMIC_RECONFIGURE_

    dynamic_reconfigure::Server< zeabus_control::PIDThrusterConfig > server_reconfigure;
    dynamic_reconfigure::Server< zeabus_control::PIDThrusterConfig >::CallbackType 
            function_reconfigue;
    function_reconfigue = boost::bind( &dynamic_reconfigure_callback , _1 , _2 );
    server_reconfigure.setCallback( function_reconfigue );
    zeabus_ros::DynamicReconfigure drh; // dynamic reconfigure handle

#endif // _DYNAMIC_RECONFIGURE_ 

    ros::Rate rate( frequency );
    drh.load( "zeabus_control" , "parameter" , "pid_thruster.yaml" , ros::this_node::getName() );
    while( ros::ok() )
    {
        lock_dynamic_reconfigure.lock(); // acquire lock part dynamic reconfigure
        if( avaliable_new_parameter )
        {
            printf( "Parameter PIDS %8.3f%8.3f%8.3f%8.3f\n" , parameter[0] , parameter[1] ,
                    parameter[2] , parameter[3] );
            avaliable_new_parameter = false;
            drh.dump( "zeabus_control" , "parameter" , "pid_thruster.yaml" 
                    , ros::this_node::getName() );
        }
        lock_dynamic_reconfigure.unlock(); // release lock part dynamic reconfigure
        rate.sleep();
    }

}
