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
    zeabus_ros::Node node( argv , argc , "control_force" );

    ros::NodeHandle ph("~"); // param handle
    ros::NodeHandle nh(""); // node handle

    PID pid[8];

    node.spin();

    const static double time_out = 3;

// =========================== PART ROS PARAM =================================
    int frequency = 30;
    ph.param< int >( "frequency" , frequency , 30 );

    std::string topic_force_target;
    ph.param< std::string >( "topic_input_target" , topic_force_target , "control/target_force" );

    std::string topic_force_input;
    ph.param< std::string >( "topic_input_force" , topic_force_input , "control/force" );

    std::string topic_force_output;
    ph.param< std::string >( "topic_output_force" , topic_force_output , "control/current_force" ); 

    std::string topic_send_throttle;
    ph.param< std::string >( "topic_output_dshot", 
            topic_send_throttle, 
            "hardware/thruster_throttle" );

    std::string topic_force_command;
    ph.param< std::string >( "topic_output_command" , topic_force_command , "control/command_force");

// =========================== PART parameter =================================

#ifdef _DYNAMIC_RECONFIGURE_

    dynamic_reconfigure::Server< zeabus_control::PIDThrusterConfig > server_reconfigure;
    dynamic_reconfigure::Server< zeabus_control::PIDThrusterConfig >::CallbackType 
            function_reconfigue;
    function_reconfigue = boost::bind( &dynamic_reconfigure_callback , _1 , _2 );
    server_reconfigure.setCallback( function_reconfigue );
    zeabus_ros::DynamicReconfigure drh; // dynamic reconfigure handle

#endif // _DYNAMIC_RECONFIGURE_ 

// =========================== PART ROS VARIABLE ============================ 
    // Part receive target force
    std::mutex lock_target_force;
    zeabus_utility::Float64Array8 message_target_force;
    zeabus_ros::subscriber::BaseClass< zeabus_utility::Float64Array8 > listener_target_force( &nh,
            &message_target_force );
    listener_target_force.setup_mutex_data( &lock_target_force );
    // Part receive current telemetry
    std::mutex lock_current_telemetry;
    zeabus_elec_ros::MessageTelemetryValue message_current_telemetry;
    zeabus_ros::subscriber::BaseClass< zeabus_elec_ros::MessageTelemetryValue > 
            listener_current_telemetry( &nh , &message_current_telemetry );
    listener_target_force.setup_mutex_data( &lock_current_telemetry );
    // Part set client for send dshot or throttle
    ros::ServiceClient client_throttle = nh.serviceClient< zeabus_utility::SendThrottle >(
            topic_send_throttle );
    zeabus_utility::ServiceDepth throttle_handle;
    zeabus_utility::Int16Array8 message_throttle;
    ros::Publisher publish_throttle = nh.advertise< zeabus_utility::Int16Array8 >(
            topic_send_throttle , 1 );
    // Part send output current force
    zeabus_utility::Float64Array8 message_current_force;
    ros::Publisher publish_current_force = nh.advertise< zeabus_utility::Float64Array8 >(
            topic_output_force , 1 ); 

// =========================== PART VARIABLE FOR ACTIVATE =======================
    std::vector< double > vector_target_force;
    vector_target_force.assign( 8 , 0);
    std::vector< double > vector_current_force;
    vector_current_force.assign( 8  , 0 );
    std::vector< int16_t > vector_throttle;
    vector_throttle.assign( 8 , 0 );
    std::vector< unsigned int32_t > vector_erpm;
    vector_erpm.assign( 8 , 0 );

    ros::Rate rate( frequency );
    ros::Time time_stamp = ros::Time::now(); // use to collect now time
    drh.load( "zeabus_control" , "parameter" , "pid_thruster.yaml" , ros::this_node::getName() );

active_main:
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
            dynamic_reconfigure_set_parameter( pid );
        }
        lock_dynamic_reconfigure.unlock(); // release lock part dynamic reconfigure
        // Because I have run 2 thread : callback thread and activate
        time_stamp = ros::Time::now();
check_target_force:
        lock_target_force.lock();
        if( ( time_stamp - message_target_force.header.stamp ).toSec() < time_out )
        {
            vector_target_force = message_target_force.data;
        } // condition use target from publish
        else
        {
            vector_target_force.assign( 8 , 0 ); 
        } // condition set zero
        lock_target_force.unlock();
check_current_force:
        lock_current_telemetry.lock();
        for( unsigned int run = 0 ; run < 8 ; run++ )
        {
            vector_erpm[ run ] = message_current_telemetry.ax_telemetry_value[ run ].erpm;
        }
        lock_current_telemetry.unlock();
        rate.sleep();
        
    }

}
