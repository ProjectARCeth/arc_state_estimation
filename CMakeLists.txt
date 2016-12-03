cmake_minimum_required(VERSION 2.8.3)
project(arc_state_estimation)

find_package(yaml_cpp_catkin QUIET)

find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS
  	roscpp
    geometry_msgs
    std_msgs
    arc_tools
    sensor_msgs
    eigen_catkin
    catkin_simple
    tf
)

catkin_simple(ALL_DEPS_REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})

cs_add_executable(attitude_filter src/attitude_filter.cpp)

cs_install()
cs_export()
