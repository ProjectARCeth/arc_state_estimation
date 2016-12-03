#include "arc_tools/coordinate_transform.hpp"
#include "arc_tools/kalman_filter.hpp"
#include "arc_tools/state_and_path_publisher.hpp"

#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/transform_broadcaster.h>

ros::Subscriber imu_sub;
//Declaration of functions.
void kalmanUpdater(const sensor_msgs::Imu::ConstPtr & imu_data);
void tfBroadcaster(const Eigen::Vector3d euler, const Eigen::Vector3d position);
//Updating Queue ~ Updating Frequency.
int queue_length = 10;
//Initialisation of kalman and publisher class.

arc_tools::KalmanFilterOrientation kalman;
arc_tools::StateAndPathPublisher pub;

int main(int argc, char** argv){
	ros::init(argc, argv, "attitudefilter");
	ros::NodeHandle node;
	pub.createPublisher(&node);
	//Getting parameters.
	getParameters(&node);
	//Initialising inital state.
	Eigen::VectorXd x_0 = Eigen::VectorXd::Zero(6,1);
	//Subscribing & Update.
	imu_sub = node.subscribe("/imu0", queue_length, kalmanUpdater);
	ros::spin();
	return 0;
}

void kalmanUpdater(const sensor_msgs::Imu::ConstPtr & imuData){
	//Updating kalman filter.
	kalman.update(imuData);
	//Publishing euler angles & pose.
	Eigen::VectorXd state(12) = Eigen::VectorXd::Zero(12,1);
	state.segment<3>(3) = kalman.getState().segment<3>(0);
	state.segment<3>(9) = kalman.getState().segment<3>(3);
	pub.publish(state, false);
	tfBroadcaster(state.segment<3>(3), state.segment<3>(0));
}	

void getParameters(ros::NodeHandle* node){
	node->getParam("/attitude_filter/StateEstimation/ImuFilter/errorStateEulerDot", error_state_euler_dot);
	node->getParam("/attitude_filter/StateEstimation/ImuFilter/errorStateLinearAcceleration", 
		error_state_linear_acceleration);
	node->getParam("/attitude_filter/StateEstimation/ImuFilter/errorMeasurementGyro", error_measurement_gyro);
	node->getParam("/attitude_filter/StateEstimation/ImuFilter/errorMeasurementLinearAccelerometer", 
		error_measurement_linear_accelerometer);
	node->getParam("/attitude_filter/StateEstimation/ImuFilter/subAndPubQueueLength", queue_length);

	std::cout << "error_state_euler_dot: " << error_state_euler_dot << std::endl;
	std::cout << "error_state_linear_acceleration: " 
		<< error_state_linear_acceleration << std::endl;
	std::cout << "error_measurement_gyro: " << error_measurement_gyro << std::endl;
	std::cout << "error_measurement_linear_accelerometer: " 
		<< error_measurement_linear_accelerometer << std::endl;
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