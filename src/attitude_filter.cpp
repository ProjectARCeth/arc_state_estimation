#include "arc_tools/coordinate_transform.hpp"
#include "arc_state_estimation/orientation_filter.hpp"
#include "arc_tools/state_and_path_publisher.hpp"

#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/transform_broadcaster.h>

ros::Subscriber imu_sub;
//Declaration of functions.
void orientUpdater(const sensor_msgs::Imu::ConstPtr & imu_data);
void tfBroadcaster(const Eigen::Vector3d euler, const Eigen::Vector3d position);
void getParameters(ros::NodeHandle* node);
//Orientation filter weight.
double orient_filter_weight;
//Initial conditions.
double init_roll;
double init_pitch;
double init_yaw;
//Updating Queue ~ Updating Frequency.
int queue_length = 10;
//Init class objects of Publisher and Filter.
arc_state_estimation::OrientationFilter orient_filter;
arc_tools::StateAndPathPublisher pub;

int main(int argc, char** argv){
	ros::init(argc, argv, "attitudefilter");
	ros::NodeHandle node;
	pub.createPublisher(&node);
	//Initialising inital state.
	Eigen::VectorXd x_0 = Eigen::VectorXd::Zero(6,1);
	x_0(0) = init_roll; x_0(1) = init_pitch; x_0(2) = init_yaw;
	orient_filter.initOrientationFilter(x_0, orient_filter_weight);
	//Subscribing & Update.
	imu_sub = node.subscribe("/imu0", queue_length, orientUpdater);
	ros::spin();
	return 0;
}

void orientUpdater(const sensor_msgs::Imu::ConstPtr & imuData){
	//Updating kalman filter.
	orient_filter.update(imuData);
	//Publishing euler angles & pose.
	Eigen::VectorXd state = Eigen::VectorXd::Zero(12,1);
	state.segment<3>(3) = orient_filter.getState().segment<3>(0);
	state.segment<3>(9) = orient_filter.getState().segment<3>(3);
	pub.publish(state, false);
	tfBroadcaster(state.segment<3>(3), state.segment<3>(0));
}	

void tfBroadcaster(const Eigen::Vector3d euler, const Eigen::Vector3d position){
  //Transform euler & position.
  geometry_msgs::Quaternion quat = transformQuaternionEuler(euler);
  //Set orientation and vector.
  tf::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
  tf::Vector3 tf_vector(position(0), position(1), position(2));
  //Setting tf - broadcast.
  static tf::TransformBroadcaster broadcaster;
  broadcaster.sendTransform(tf::StampedTransform(
        tf::Transform(tf_quat, tf_vector), ros::Time::now(),"world", "velodyne"));
}

void getParameters(ros::NodeHandle* node){
	node->getParam("/attitude_filter/StateEstimation/weight_orientation", weight);
	node->getParam("/attitude_filter/StateEstimation/init_roll", init_roll);
	node->getParam("/attitude_filter/StateEstimation/init_pitch", init_pitch);
	node->getParam("/attitude_filter/StateEstimation/init_yaw", init_yaw);
}	