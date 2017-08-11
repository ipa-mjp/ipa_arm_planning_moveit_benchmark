cmake_minimum_required(VERSION 2.8.3)
project(gstomp_benchmark)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  stomp_moveit
  roscpp
  rospy
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
	INCLUDE_DIRS include
#	LIBRARIES ${LIB_NAME}
	CATKIN_DEPENDS moveit_core moveit_ros_planning stomp_moveit roscpp rospy std_msgs
	DEPENDS
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(gstomp_run_benchmark src/RunBenchmark.cpp)
add_dependencies(gstomp_run_benchmark ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gstomp_run_benchmark ${LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
