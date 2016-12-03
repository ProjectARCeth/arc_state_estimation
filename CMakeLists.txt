cmake_minimum_required(VERSION 2.8.3)
project(arc_state_estimation)

find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS
  	roscpp
    geometry_msgs
    std_msgs
    arc_tools
    sensor_msgs
    eigen_catkin
    catkin_simple REQUIRED
    tf
)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME} 
	src/orientation_filter.cpp
)

cs_add_executable(attitude_filter src/attitude_filter.cpp)
target_link_libraries(attitude_filter ${catkin_LIBRARIES} ${PROJECT_NAME})

cs_install()
cs_export()