#ifndef STATE_AND_PATH_PUBLISHER_ARC_TOOLS_HPP
#define STATE_AND_PATH_PUBLISHER_ARC_TOOLS_HPP

#include "arc_msgs/State.h"
#include "arc_tools/coordinate_transform.hpp"

#include "ros/ros.h"
#include "Eigen/Dense"
#include <fstream>
#include <iostream>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Path.h"

namespace arc_state_estimation{

class StateAndPathPublisher {
public:
	StateAndPathPublisher(std::string pub_name, std::string filename);
	void createPublisher(ros::NodeHandle* node);
	//State (x,y,z  phi,theta,psi, x_dot,y_dot,z_dot, phi_dot,theta_dot,psi_dot, ...).
	void publishWithEuler(Eigen::Vector3d position, Eigen::Vector3d euler, 
						  Eigen::Vector3d lin_vel, Eigen::Vector3d ang_vel, bool stop);
	void publishWithQuaternion(Eigen::Vector3d position, Eigen::Vector4d quat, 
						       Eigen::Vector3d lin_vel, Eigen::Vector3d ang_vel, bool stop);

private:
    ros::Publisher pub_state_;
    ros::Publisher pub_path_;
	int array_position_;
	arc_msgs::State state_;
	std::vector<geometry_msgs::PoseStamped> path_vector_;
	nav_msgs::Path path_;
	std::string pub_name_;
	std::ofstream stream_;
};
}//namespace arc_state_estimation.

#endif