cmake_minimum_required(VERSION 2.8.3)
project(simulation_environment)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	gazebo_ros
)

# Depending on system install of Gazebo
find_package(gazebo REQUIRED)
catkin_package(
	CATKIN_DEPENDS
	roscpp
	rospy
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
	${catkin_INCLUDE_DIRS}
	${src}
	${GAZEBO_INCLUDE_DIR}
)
# Gazebo set flags
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
