#include "arc_tools/kalman_filter.hpp"
#include "arc_tools/state_and_path_publisher.hpp"

#include "ros/ros.h"

#include "yaml-cpp/yaml.h"

ros::Subscriber imu_sub;
//Declaration of functions.
void kalmanUpdater(const sensor_msgs::Imu::ConstPtr & imu_data);
void getParameters(ros::NodeHandle* node);
//Definition of errors
double error_state_euler_dot;
double error_state_linear_acceleration;
double error_measurement_linear_accelerometer;
double error_measurement_gyro;
//Updating Queue ~ Updating Frequency.
int queue_length = 50;
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
	Eigen::VectorXd x_0(15);
	x_0[0]= 0.0; x_0[1]=0.0; x_0[2]=0.0;	//Positions.
	x_0[3]= M_PI/2; x_0[4]=0.0; x_0[5]=0.0;	//Euler angles.
	x_0[6]= 0.0; x_0[7]=0.0; x_0[8]=0.0;	//Linear velocities.
	x_0[9]= 0.0; x_0[10]=0.0; x_0[11]=0.0;	//Angular velocities.
	x_0[12]= 0.0; x_0[13]=0.0; x_0[14]=0.0;	//Linear accelerations.
	//Initialising Kalman Filter.
	kalman.initWithErrors(x_0, 
					error_state_euler_dot, error_state_linear_acceleration,
                    error_measurement_gyro, error_measurement_linear_accelerometer);
	//Subscribing & Update.
	imu_sub = node.subscribe("/imu0", queue_length, kalmanUpdater);
	ros::spin();
	return 0;
}

void kalmanUpdater(const sensor_msgs::Imu::ConstPtr & imuData){
	//Updating kalman filter.
	kalman.update(imuData);
	//Publishing euler angles & pose.
	pub.publish(kalman.getState(), false);
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
