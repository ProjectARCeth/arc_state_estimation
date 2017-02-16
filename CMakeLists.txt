cmake_minimum_required(VERSION 2.8.3)
project(arc_state_estimation)

find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS
  	roscpp
  	roslib
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
    src/car_model.cpp
)

cs_add_executable(state_estimation src/state_estimation.cpp)
target_link_libraries(state_estimation ${catkin_LIBRARIES}  ${PROJECT_NAME})

cs_install()
cs_export()