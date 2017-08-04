cmake_minimum_required(VERSION 2.8.3)
project(ipa_arm_planning_moveit_benchmark)

set(IPA_LIB_NAME ipa_benchmarks)

add_compile_options(-std=c++11)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_ros_planning
  moveit_ros_warehouse
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
  LIBRARIES ${IPA_LIB_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs moveit_ros_planning moveit_ros_warehouse
  DEPENDS
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${CMAKE_CURRENT_LIST_DIR}/include)
link_directories(${catkin_LIBRARY_DIRS})

add_library(${IPA_LIB_NAME} src/BenchmarkOptions.cpp
                               src/BenchmarkExecutor.cpp)

set_target_properties(${IPA_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${IPA_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(ipa_run_benchmark src/RunBenchmark.cpp)
target_link_libraries(ipa_run_benchmark ${IPA_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(
  TARGETS
    ${IPA_LIB_NAME} ipa_run_benchmark
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})