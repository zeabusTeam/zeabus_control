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
    listener_target_force.setup_subscriber( topic_force_target , 1 );
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
            topic_force_output , 1 ); 

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
            vector_target_force.assign( 0 ); 
        } // condition set zero
        lock_target_force.unlock();
check_current_force:
        lock_current_telemetry.lock();
        for( unsigned int run = 0 ; run < 8 ; run++ )
        {
            vector_erpm[ run ] = message_current_telemetry.ax_telemetry_value[ run ].erpm;
            message_current_force.header.stamp = message_current_telemetry.header.stamp;
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
            if( fabs( vector_addition_throttle.at( run ) ) > 250 )
            {
                vector_addition_throttle.at( run ) = 
                        copysign( 250 , vector_addition_throttle.at( run ) );
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
        message_throttle.header.stamp = time_stamp;
        message_throttle.data = vector_throttle;
        publish_throttle.publish( message_throttle );
send_throttle:
        srv_throttle.request.header = message_throttle.header;
        srv_throttle.request.data = message_throttle.data;
        if( client_throttle.call( srv_throttle ) )
        {
            print_reported( &vector_target_force , &vector_current_force,
                    &vector_addition_throttle , &vector_throttle , 
                    &vector_erpm );
        }
        else
        {
            ROS_ERROR( "Falied to call send throttle");
        }
        rate.sleep();

    }

}

void compare_data( const boost::array< int16_t , 8 >* vector_throttle,
        const boost::array< uint32_t , 8 >* vector_erpm,
        const double* positive_table,
        const double* negative_table,
        boost::array< double , 8 >* current_force )
{
    for( unsigned int run = 0 ; run < 8 ; run++ )
    {
        int position = vector_erpm->at( run ) / 7;
        if( position > 4000 )
        {
            std::cout   << zeabus::escape_code::bold_yellow << "Warning! erpm are over by pos is "
                        << position << zeabus::escape_code::normal_white
                        << " for thruster number " << run << "\n";
            ros::shutdown();
        } // condition ohmy god
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
        const boost::array< uint32_t , 8 >* erpm )
{
    printf( "================== CONTROL REPORTED ===============");
    printf( "No.|   target|  current| addition|   output|     erpm\n");
    for( unsigned int run = 0 ; run <  8 ; run++ )
    {
        printf( "%3d|%9.3f|%9.3f|%9.3f|%9d,%9d\n" , run , target_force->at( run ), 
                current_force->at( run ) , addition_throttle->at( run ) ,
                throttle->at( run ) , erpm->at( run ) );
    }
}
