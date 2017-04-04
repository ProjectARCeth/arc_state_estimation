#ifndef CAR_MODEL_ARC_TOOLS_HPP
#define CAR_MODEL_ARC_TOOLS_HPP

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
	CarModel();
	~CarModel();	
	void createPublisher(ros::NodeHandle* node);
	void updateModel(Eigen::Vector4d orientation);
	double getVelocity();
	void setDistanceWheelAxis(float distance_axis);
	void setLengthWheelAxis(float length_axis);
	void setSteeringAngle(float steering_angle);
	void setVelocityLeft(float velocity_left);
	void setVelocityRight(float velocity_right);

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