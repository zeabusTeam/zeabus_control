cmake_minimum_required( VERSION 2.8.3 )
project( zeabus_control )

find_package( catkin REQUIRED COMPONENTS 
    roscpp
    nav_msgs
    geometry_msgs
    zeabus_utility
    zeabus_elec_ros
    dynamic_reconfigure
)

find_package( Boost REQUIRED COMPONENTS )
find_package( zeabus_library REQUIRED COMPONENTS )

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${zeabus_library_INCLUDE_DIRS}
    "${PROJECT_SOURCE_DIR}/include"
)

generate_dynamic_reconfigure_options(
    cfg/PIDandSaturation.cfg
    cfg/PIDnonDiscrete.cfg
)

catkin_package(
    CATKIN_DEPENDS roscpp
    DEPENDS Boost ${PROJECT_NAME}_gencfg
)

