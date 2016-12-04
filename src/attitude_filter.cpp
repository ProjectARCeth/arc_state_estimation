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

<<<<<<< HEAD
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
=======
int main(int argc, char** argv) {
  ros::init(argc, argv, "attitudefilter");
  ros::NodeHandle node;
  pub.createPublisher(&node);
  //Getting parameters.
  getParameters(&node);
  //Initialising inital state.
  Eigen::VectorXd x_0(15);
  x_0[0] = 0.0;
  x_0[1] = 0.0;
  x_0[2] = 0.0;  //Positions.
  x_0[3] = 0.0;
  x_0[4] = 0.0;
  x_0[5] = 0.0;  //Euler angles.
  x_0[6] = 0.0;
  x_0[7] = 0.0;
  x_0[8] = 0.0;  //Linear velocities.
  x_0[9] = 0.0;
  x_0[10] = 0.0;
  x_0[11] = 0.0;	//Angular velocities.
  x_0[12] = 0.0;
  x_0[13] = 0.0;
  x_0[14] = 0.0;  //Linear accelerations.
  //Initialising Kalman Filter.
  kalman.initWithErrors(x_0, error_state_euler_dot,
                        error_state_linear_acceleration, error_measurement_gyro,
                        error_measurement_linear_accelerometer);
  //Subscribing & Update.
  imu_sub = node.subscribe("/imu0", queue_length, kalmanUpdater);
  ros::spin();
  return 0;
}

void kalmanUpdater(const sensor_msgs::Imu::ConstPtr & imuData) {
  //Updating kalman filter.
  kalman.update(imuData);
  //Publishing euler angles & pose.
  pub.publish(kalman.getState(), false);
  tfBroadcaster(kalman.getState().segment < 3 > (3),
                kalman.getState().segment < 3 > (0));
}

void getParameters(ros::NodeHandle* node) {
  node->getParam(
      "/attitude_filter/StateEstimation/ImuFilter/errorStateEulerDot",
      error_state_euler_dot);
  node->getParam(
      "/attitude_filter/StateEstimation/ImuFilter/errorStateLinearAcceleration",
      error_state_linear_acceleration);
  node->getParam(
      "/attitude_filter/StateEstimation/ImuFilter/errorMeasurementGyro",
      error_measurement_gyro);
  node->getParam(
      "/attitude_filter/StateEstimation/ImuFilter/errorMeasurementLinearAccelerometer",
      error_measurement_linear_accelerometer);
  node->getParam(
      "/attitude_filter/StateEstimation/ImuFilter/subAndPubQueueLength",
      queue_length);

  std::cout << "error_state_euler_dot: " << error_state_euler_dot << std::endl;
  std::cout << "error_state_linear_acceleration: "
      << error_state_linear_acceleration << std::endl;
  std::cout << "error_measurement_gyro: " << error_measurement_gyro
      << std::endl;
  std::cout << "error_measurement_linear_accelerometer: "
      << error_measurement_linear_accelerometer << std::endl;
}

void tfBroadcaster(const Eigen::Vector3d euler,
                   const Eigen::Vector3d position) {
>>>>>>> 1082abfe0d14ff73b06f13734877ac410c212d11
  //Transform euler & position.
  geometry_msgs::Quaternion quat = transformQuaternionEuler(euler);
  //Set orientation and vector.
  tf::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
  tf::Vector3 tf_vector_1(position(0), position(1), position(2));
  //Setting tf - broadcast from odom to rear_axle
  static tf::TransformBroadcaster broadcaster;
<<<<<<< HEAD
  broadcaster.sendTransform(tf::StampedTransform(
        tf::Transform(tf_quat, tf_vector), ros::Time::now(),"world", "velodyne"));
}

void getParameters(ros::NodeHandle* node){
	node->getParam("/attitude_filter/StateEstimation/weight_orientation", weight);
	node->getParam("/attitude_filter/StateEstimation/init_roll", init_roll);
	node->getParam("/attitude_filter/StateEstimation/init_pitch", init_pitch);
	node->getParam("/attitude_filter/StateEstimation/init_yaw", init_yaw);
}	
=======
  broadcaster.sendTransform(
      tf::StampedTransform(tf::Transform(tf_quat, tf_vector_1),
                           ros::Time::now(), "odom", "rear_axle"));
  //Setting tf - broadcast from rear_axle to velodyne
  Eigen::Matrix3d M = getRotationMatrix(euler);
  Eigen::Vector3d initial_vector(0.5786, 0.0, 1.0705);
  Eigen::Vector3d new_vector = M * initial_vector;
  tf::Vector3 tf_vector_2(new_vector(0), new_vector(1), new_vector(2));
  broadcaster.sendTransform(
      tf::StampedTransform(tf::Transform(tf_quat, tf_vector_2),
                           ros::Time::now(), "rear_axle", "velodyne"));
}
>>>>>>> 1082abfe0d14ff73b06f13734877ac410c212d11
