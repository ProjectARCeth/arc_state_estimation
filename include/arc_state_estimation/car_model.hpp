#ifndef CAR_MODEL_ARC_TOOLS_HPP
#define CAR_MODEL_ARC_TOOLS_HPP

#include "arc_msgs/State.h"
#include "arc_tools/coordinate_transform.hpp"

#include <iostream>
#include "Eigen/Dense"
#include "ros/ros.h"

#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"

namespace arc_state_estimation{

class CarModel{
public:
	CarModel(float distance_wheels, float length_axis);
	void createPublisher(ros::NodeHandle* node);
	void updateModel(Eigen::Vector4d orientation);
	void set_steering_angle(float steering_angle);
	void set_velocity_left(float velocity_left);
	void set_velocity_right(float velocity_right);

private:
	ros::Publisher pub_velocity_;
	float L_;
	float B_;
	float steering_angle_;
	float velocity_left_;
	float velocity_right_;
};
}//namespace arc_state_estimation.

#endif