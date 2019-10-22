cmake_minimum_required( VERSION 2.8.3 )
project( zeabus_control )

find_package( catkin REQUIRED COMPONENTS 
    roscpp
    nav_msgs
    geometry_msgs
    zeabus_utility
)

find_package( Boost REQUIRED COMPONENTS )
find_package( zeabus_library REQUIRED COMPONENTS )

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${zeabus_library_INCLUDE_DIRS}
)

catkin_package(
    CATKIN_DEPENDS roscpp
    DEPENDS Boost
)

add_executable( control src/main.cpp )
target_link_libraries( control 
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${zeabus_library_LIBRARIES}
)
