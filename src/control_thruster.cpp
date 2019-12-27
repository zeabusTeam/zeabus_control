// FILE			: control_thruster.cpp
// AUTHOR		: K.Supasan
// CREATE ON	: 2019, December 23 
// MAINTAINER	: K.Supasan

// MACRO DETAIL

// README

// REFERENCE

// MACRO SET
#define _DYNAMIC_RECONFIGURE_
#define _PUBLISH_ERROR_
#define _PRINT_REPORTES_

// MACRO CONDITION

#include    <control_thruster.hpp>

int main( int argv , char** argc )
{
    zeabus_ros::Node node( argv , argc , "control_force" );

    ros::NodeHandle ph("~"); // param handle
    ros::NodeHandle nh(""); // node handle

    PID pid[8];

    node.spin();

    const static double time_out = 1.0;

// =========================== PART ROS PARAM =================================
    int frequency = 30;
    ph.param< int >( "frequency" , frequency , 30 );

    std::string topic_force_target;
    ph.param< std::string >( "topic_input_target" , topic_force_target , "control/force/target" );

    std::string topic_force_absolute;
    ph.param< std::string >( "topic_input_force" , topic_force_absolute , "control/force/absolute" );

    std::string topic_force_current;
    ph.param< std::string >( "topic_output_force" , topic_force_current , "control/force/current" ); 

#ifdef _PUBLISH_ERROR_
    std::string topic_force_error;;
    ph.param< std::string >( "topic_output_error" , topic_force_error , "control/force/error" );
    boost::array< double , 8 > vector_error_force; 
#endif
    std::string topic_send_throttle;
    ph.param< std::string >( "topic_output_dshot", 
            topic_send_throttle, 
            "hardware/thruster_throttle" );

    std::string topic_telemetry_input;
    ph.param< std::string >( "topic_telemetry" , topic_telemetry_input , "elec/telemetry_value");

// =========================== PART parameter =================================

#ifdef _DYNAMIC_RECONFIGURE_

    dynamic_reconfigure::Server< zeabus_control::PIDThrusterConfig > server_reconfigure;
    dynamic_reconfigure::Server< zeabus_control::PIDThrusterConfig >::CallbackType 
            function_reconfigue;
    function_reconfigue = boost::bind( &dynamic_reconfigure_callback , _1 , _2 );
    server_reconfigure.setCallback( function_reconfigue );
    zeabus_ros::DynamicReconfigure drh; // dynamic reconfigure handle

#endif // _DYNAMIC_RECONFIGURE_ 

    double* positive_table, *negative_table;
    int* temporay_table;
    unsigned int num_line;
    zeabus::FileCSV fh;

    fh.open( zeabus_ros::get_full_path( "zeabus_control" , "parameter" , "rpm_forward_table.csv") );
    (void)fh.count_line( &num_line );
    positive_table = ( double*) malloc( sizeof( double ) * num_line );
    temporay_table = ( int* ) malloc( sizeof( int ) * num_line );
    zeabus::extract_csv_type_2( &fh , temporay_table , positive_table );
    free( temporay_table );
    fh.close();
    
    fh.open( zeabus_ros::get_full_path( "zeabus_control" , "parameter" , "rpm_reverse_table.csv") );
    (void)fh.count_line( &num_line );
    negative_table = ( double*) malloc( sizeof( double ) * num_line );
    temporay_table = ( int* ) malloc( sizeof( int ) * num_line );
    zeabus::extract_csv_type_2( &fh , temporay_table , negative_table );
    free( temporay_table );
    fh.close();

// =========================== PART ROS VARIABLE ============================ 
    // Part receive target force
    std::mutex lock_target_force;
    zeabus_utility::Float64Array8 message_target_force;
    zeabus_ros::subscriber::BaseClass< zeabus_utility::Float64Array8 > listener_target_force( &nh,
            &message_target_force );
    listener_target_force.setup_mutex_data( &lock_target_force );
    listener_target_force.setup_subscriber_timestamp( topic_force_target , 1 );

    // Part receive absolute force
    std::mutex lock_absolute_force;
    zeabus_utility::Float64Array8 message_absolute_force;
    zeabus_ros::subscriber::BaseClass< zeabus_utility::Float64Array8 > listener_absolute_force( &nh,
            &message_absolute_force );
    listener_absolute_force.setup_mutex_data( &lock_absolute_force );
    listener_absolute_force.setup_subscriber_timestamp( topic_force_absolute , 1 );

    // Part receive current telemetry
    std::mutex lock_current_telemetry;
    zeabus_elec_ros::MessageTelemetryValue message_current_telemetry;
    zeabus_ros::subscriber::BaseClass< zeabus_elec_ros::MessageTelemetryValue > 
            listener_current_telemetry( &nh , &message_current_telemetry );
    listener_current_telemetry.setup_mutex_data( &lock_current_telemetry );
    listener_current_telemetry.setup_subscriber( topic_telemetry_input , 1 );

    // Part set client for send dshot or throttle
    ros::ServiceClient client_throttle = nh.serviceClient< zeabus_utility::SendThrottle >(
            topic_send_throttle );
    zeabus_utility::SendThrottle srv_throttle;
    zeabus_utility::Int16Array8 message_throttle;
    ros::Publisher publish_throttle = nh.advertise< zeabus_utility::Int16Array8 >(
            topic_send_throttle , 1 );

    // Part send output current force
    zeabus_utility::Float64Array8 message_current_force;
    ros::Publisher publish_current_force = nh.advertise< zeabus_utility::Float64Array8 >(
            topic_force_current , 1 ); 

#ifdef _PUBLISH_ERROR_
    // Part send output error force
    zeabus_utility::Float64Array8 message_error_force;
    ros::Publisher publish_error_force = nh.advertise< zeabus_utility::Float64Array8 >(
            topic_force_error , 1 ); 
#endif // _PUBLISH_ERROR_

// =========================== PART VARIABLE FOR ACTIVATE =======================
    boost::array< double , 8 > vector_addition_throttle;
    vector_addition_throttle.assign( 0 );
    boost::array< double , 8 > vector_target_force;
    vector_target_force.assign( 0 );
    boost::array< double , 8 > vector_current_force;
    vector_current_force.assign( 0 );
    boost::array< int16_t , 8 > vector_throttle;
    vector_throttle.assign( 0 );
    boost::array< uint32_t , 8 > vector_erpm;
    vector_erpm.assign( 0 );

    ros::Rate rate( frequency );
    ros::Time time_stamp = ros::Time::now(); // use to collect now time
    drh.load( "zeabus_control" , "parameter" , "pid_thruster.yaml" , ros::this_node::getName() );

    bool thruster_state = true;
    unsigned int count_thruster;

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
        lock_absolute_force.lock();
        if( ( time_stamp - message_absolute_force.header.stamp ).toSec() < time_out  )
        {
            vector_target_force = message_absolute_force.data;
        }
        else if( ( time_stamp - message_target_force.header.stamp ).toSec() < time_out )
        {
            vector_target_force = message_target_force.data;
        } // condition use target from publish
        else
        {
            vector_target_force.assign( 0 ); 
        } // condition set zero
        lock_absolute_force.unlock();
        lock_target_force.unlock();
check_current_force:
        lock_current_telemetry.lock();
        count_thruster = 8 ;
        thruster_state = true;
        for( unsigned int run = 0 ; run < 8 ; run++ )
        {
            vector_erpm[ run ] = message_current_telemetry.ax_telemetry_value[ run ].erpm;
            message_current_force.header.stamp = message_current_telemetry.header.stamp;
            if( message_current_telemetry.ax_telemetry_value[ run ].temperature == 0 )
            {
                count_thruster -= 1;
            }
        }
        if( thruster_state && count_thruster == 0 )
        {
            thruster_state = false;
        }
        lock_current_telemetry.unlock();
        compare_data( &vector_throttle , &vector_erpm , //  previous data and new data
                positive_table , negative_table , // table for find force
                &vector_current_force ); // result
        message_current_force.data = vector_current_force;
        publish_current_force.publish( message_current_force ); // publish current force
compute_throttle_force:
        // Will run loop for compute control get throttle
        for( unsigned int run = 0 ; run < 8 ; run++ )
        {
            vector_addition_throttle.at( run ) = pid[ run ].calculate(
                    vector_target_force.at( run ) - vector_current_force.at( run ),
                    message_current_force.header.stamp,
                    0 ); // saturation = 0
#ifdef _PUBLISH_ERROR_
            vector_error_force.at( run ) = 
                    vector_target_force.at( run ) - vector_current_force.at( run ); 
#endif // _PUBLISH_ERROR_
            if( fabs( vector_addition_throttle.at( run ) ) > 250 )
            {
                vector_addition_throttle.at( run ) = 
                        copysign( 250 , vector_addition_throttle.at( run ) );
            }
            else if( fabs( vector_addition_throttle.at( run ) ) < 1  )
            {
                vector_addition_throttle.at( run ) = 
                        copysign( 1 , vector_addition_throttle.at( run ) );
            }
            else
            {
                ;
            }
            if( equal( vector_target_force.at( run ) , 0 ) )
            {
                vector_throttle.at( run ) = 0;
            }
            else
            {
                vector_throttle.at( run ) += vector_addition_throttle.at( run );
            }
 
            if( fabs( vector_throttle.at( run ) ) > 999 )
            {
                vector_throttle.at( run ) = copysign( 1000 , vector_throttle.at( run ) );
            }
        }
        if( ! thruster_state )
        {
            for( unsigned int run = 0 ; run < 8 ; run++ )
            {
                pid[ run ].reset();
                vector_throttle.at( run ) = 0;
            }
        }
#ifdef _PUBLISH_ERROR_
        message_error_force.header.stamp = time_stamp;
        message_error_force.data = vector_error_force;
        publish_error_force.publish( message_error_force );
#endif // _PUBLISH_ERROR_
        message_throttle.header.stamp = time_stamp;
        message_throttle.data = vector_throttle;
        publish_throttle.publish( message_throttle );
send_throttle:
        srv_throttle.request.header = message_throttle.header;
        srv_throttle.request.data = message_throttle.data;
        if( client_throttle.call( srv_throttle ) && thruster_state )
        {
#ifdef _PRINT_REPORTES_
            print_reported( &vector_target_force , &vector_current_force,
                    &vector_addition_throttle , &vector_throttle , 
                    &vector_erpm , thruster_state );
#endif 
        }
        else
        {
            ROS_ERROR( "Falied to call send throttle");
            ros::Duration( 1 ).sleep();
        }
        rate.sleep();

    }

}

// Return true in case thruster switch is active
void compare_data( const boost::array< int16_t , 8 >* vector_throttle,
        const boost::array< uint32_t , 8 >* vector_erpm,
        const double* positive_table,
        const double* negative_table,
        boost::array< double , 8 >* current_force )
{
    for( unsigned int run = 0 ; run < 8 ; run++ )
    {
        unsigned int position = vector_erpm->at( run ) / 7;
        if( position > 4000 )
        {
            std::cout   << zeabus::escape_code::bold_yellow << "Warning! erpm are over by pos is "
                        << position << zeabus::escape_code::normal_white
                        << " for thruster number " << run << " erpm is " << vector_erpm->at( run ) 
                        << "\n" ;
            ROS_WARN( "Warning! erpm over num_thruster:original_erpm:value %2d :%7d :%6d",
                    run , vector_erpm->at( run ) , position );
            // ros::shutdown();
        } // condition ohmy god
        else if( fabs( vector_throttle->at( run ) ) < 50 )
        {
            current_force->at( run ) = 0;
        }
        else if( vector_throttle->at( run ) > 0 )
        {
            current_force->at( run ) = positive_table[ position ];
        } // condition positive force of thruster
        else 
        {
            current_force->at( run ) = negative_table[ position ];
        } // condition negative force of thruster
    }
}

void print_reported(
        const boost::array< double , 8 >* target_force,
        const boost::array< double , 8 >* current_force,
        const boost::array< double , 8 >* addition_throttle,
        const boost::array< int16_t , 8 >* throttle,
        const boost::array< uint32_t , 8 >* erpm ,
        const bool state )
{
    printf( "================== CONTROL REPORTED ===============");
    printf( "\nNo.|   target|  current| addition|   output|     erpm\n");
    if( state )
    {
        for( unsigned int run = 0 ; run <  8 ; run++ )
        {
            printf( "%3d|%9.3f|%9.3f|%9.3f|%9d,%9d\n" , run , target_force->at( run ), 
                    current_force->at( run ) , addition_throttle->at( run ) ,
                    throttle->at( run ) , erpm->at( run ) );
        }
    }
    else
    {
        printf( "=================== STATE FALSE ================= " );
    }
}
