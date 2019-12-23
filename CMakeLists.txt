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
    cfg/PIDThruster.cfg
)

catkin_package(
    CATKIN_DEPENDS roscpp dynamic_reconfigure
    DEPENDS Boost ${PROJECT_NAME}_gencfg zeabus_library
)


add_executable( control_thruster 
    src/pid.cpp
    src/control_thruster_reconfigure.cpp
    src/control_thruster.cpp
)
add_dependencies( control_thruster 
    ${PROJECT_NAME}_gencfg
)
target_link_libraries(  control_thruster
    ${zeabus_library_LIBRARIES}
    ${catkin_LIBRARIES}
)
