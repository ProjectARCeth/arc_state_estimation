#include "arc_header/coordinate_transform.h"
#include "arc_header/kalman_filter.h"

#include <cmath>
#include <eigen3/Eigen/Eigen>

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"

//Probleme:
// 1. typedef--> error: not declared in scope
// 2. Implementation von Methoden einer Klasse in c++ File ?
// 3. KalmanFilter_Orientation kalman; while ... kalman() did not, why ??

//typedef 
// typedef Eigen::MatrixXd::Identity(3) Identity_3;
// typedef Eigen::MatrixXd::Identity(6) Identity_6;
// typedef Eigen::MatrixXd::Identity(9) Identity_9;
// typedef Eigen::MatrixXd::Identity(15) Identity_15;

ros::Subscriber imu_sub;
ros::Publisher orientation_pub;

void kalman_updater(const sensor_msgs::Imu::ConstPtr & imu_data);

//Definition of errors
const float error_state_euler_dot = 40;
const float error_state_linear_acceleration = 10;
const float error_measurement_linear_accelerometer = 1;
const float error_measurement_gyro = 0.001;

//Time difference
double t_current = 0.0;
double t_last = 0.0;
double dt = 0.0;
bool first_step = true;

//Initialization for kalman Update (global: no new initialization every step)
Eigen::Vector3d current_angles;
Eigen::VectorXd current_state;
geometry_msgs::Quaternion current_orientation;

//Declaration Kalman Filter
KalmanFilter_Orientation kalman;

int main(int argc, char** argv){

	ros::init(argc, argv, "attitudefilter");
	ros::NodeHandle node;

	//Initialising state matrices
	Eigen::Matrix<double,15,15> A = Eigen::MatrixXd::Identity(15,15);
	A.block<9,9>(0,6) = Eigen::MatrixXd::Identity(9,9) * dt;
	A.block<3,3>(12,12) = Eigen::MatrixXd::Zero(3,3);

	const Eigen::Matrix<double,15,15> B = Eigen::MatrixXd::Zero(15,15);

	Eigen::Matrix<double,6,15> C = Eigen::MatrixXd::Zero(6,15);

	//Initialising error matrices
	Eigen::Matrix<double,15,15> R = Eigen::MatrixXd::Identity(15,15);
	R.block<3,3>(9,9) = Eigen::MatrixXd::Identity(3,3) * error_state_euler_dot;
	R.block<3,3>(12,12) = Eigen::MatrixXd::Identity(3,3) * error_state_linear_acceleration;

	Eigen::Matrix<double,6,6> Q = Eigen::MatrixXd::Identity(6,6);
	Q.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3,3) * error_measurement_linear_accelerometer;
	Q.block<3,3>(3,3) = Eigen::MatrixXd::Identity(3,3) * error_measurement_gyro;


	//Initialising inital state
	const Eigen::VectorXd x0 = Eigen::VectorXd::Zero(9,9);
	const Eigen::Vector3d init_angles(x0(3), x0(4), x0(5));

	//Initialising Kalman Filter
	kalman.init(x0,A,B,C,Q,R);

	//Subscribing & Update loop
	while(ros::ok()){

		imu_sub = node.subscribe("/imu0", 1, kalman_updater);
		orientation_pub = node.advertise<geometry_msgs::Quaternion>("/current_orientation", 1);
		ros::spinOnce();
	}

	return 0;
}


void kalman_updater(const sensor_msgs::Imu::ConstPtr & imu_data){

	//Time Interval
	if (first_step){ 
		t_last = imu_data->header.stamp.toNSec();
		first_step = false;
	}
	t_current = imu_data->header.stamp.toNSec();
	dt = pow(t_current - t_last, -9.0);											//Transforming nanoseconds into seconds
	t_last = t_current;

	//Updating kalman filter
	kalman.update(dt, imu_data);

	//Transforming into quaternions
	current_state = kalman.state();
	current_angles(0) = current_state(3);
	current_angles(1) = current_state(4);
	current_angles(2) = current_state(5);

	current_orientation = transform_quaternion_euler(current_angles);
	orientation_pub.publish(current_orientation);
}




