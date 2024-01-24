cmake_minimum_required(VERSION 2.8.3)
project(commander)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    geometry_msgs
    std_msgs
    nav_msgs
    visualization_msgs
)

catkin_package(
    CATKIN_DEPENDS
    nav_msgs
    visualization_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${src}
 )

add_executable(navigation_robot_viz src/navigation_robot_viz.cpp)
target_link_libraries(navigation_robot_viz ${catkin_LIBRARIES})
add_dependencies(navigation_robot_viz ${catkin_EXPORTED_TARGETS})
