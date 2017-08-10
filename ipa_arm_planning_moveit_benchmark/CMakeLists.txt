cmake_minimum_required(VERSION 2.8.3)
project(ipa_arm_planning_moveit_benchmark)
add_compile_options(-std=c++11)
set(catkin_RUN_PACKAGES
	actionlib
	actionlib_msgs
	geometry_msgs
	moveit_core
	moveit_ros_perception
	moveit_ros_planning
	moveit_ros_planning_interface
	pluginlib
	roscpp
	roslib
	rospy
	sensor_msgs
	std_msgs
	std_srvs
	tf
	tf_conversions
)

set(catkin_BUILD_PACKAGES 
	${catkin_RUN_PACKAGES}
	cmake_modules
)

find_package(catkin REQUIRED COMPONENTS
	${catkin_BUILD_PACKAGES} # this makes ${catkin_LIBRARIES} include all libraries of ${catkin_BUILD_PACKAGES}
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED system filesystem date_time thread)


catkin_package(
	INCLUDE_DIRS
		include	
	CATKIN_DEPENDS
		${catkin_RUN_PACKAGES}  
	DEPENDS
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
)


add_executable(ipa_run_benchmark src/BenchmarkOptions.cpp src/BenchmarkExecutor.cpp  src/RunBenchmark.cpp)
add_dependencies(ipa_run_benchmark ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ipa_run_benchmark ${catkin_LIBRARIES})
