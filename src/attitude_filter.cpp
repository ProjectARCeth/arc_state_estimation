#include "arc_header/coordinate_transform.h"
#include "arc_header/kalman_filter.h"

#include <cmath>
#include <eigen3/Eigen/Eigen>

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"

typedef Eigen::Matrix3d Matrix_3;


ros::Subscriber imu_sub;
ros::Publisher orientation_pub;

void kalman_updater(const sensor_msgs::Imu::ConstPtr & imu_data);

//Earth constant
float g =  9.80665;

//Definition of errors
const float error_state_euler_dot = 40;
const float error_state_linear_acceleration = 10;
const float error_measurement_linear_accelerometer = 1;
const float error_measurement_gyro = 0.001;

//Time difference
double t_current = 0.0;
double t_last = 0.0;
double dt = 0.0;
int counter = 0;


//Initialization for kalman Update (global: no new initialization every step)
Eigen::Vector3d current_angles;
Eigen::VectorXd current_state;
Eigen::VectorXd current_measurements;
geometry_msgs::Quaternion current_orientation;

int main(int argc, char** argv){

	ros::init(argc, argv, "attitudefilter");
	ros::NodeHandle node;
	
	//Initialising state matrices
	Eigen::Matrix<double,15,15> A = Eigen::Matrix<double,15,15>::Identity(15);
	A.segment<9,9>(0,6) = Eigen::MatrixXd::Identity(9) * dt;
	A.segment<3,3>(12,12) = Eigen::MatrixXd::Zeros(3);

	const Eigen::Matrix<double,15,15> B = Eigen::Matrix<double,15,15>::Zero(15);

	Eigen::Matrix<double,6,15> C = Eigen::Matrix<double,6,15>::Zero(6,15);
	C.segment<3,3>(0,12) = get_rotation_matrix(init_angles);

	//Initialising error matrices
	Eigen::Matrix<double,15,15> R = Eigen::MatrixXd::Identity(15);
	R.segment<3,3>(9,9) = Eigen::MatrixXd::Identity(3) * error_state_euler_dot;
	R.segment<3,3>(12,12) = Eigen::MatrixXd::Identity(3) * error_state_linear_acceleration;

	Eigen::Matrix<double,6,6> Q = Eigen::MatrixXd::Identity(6);
	Q.segment<3,3>(0,0) = Eigen::MatrixXd::Identity(3) * error_measurement_linear_accelerometer;
	Q.segment<3,3>(3,3) = Eigen::MatrixXd::Identity(3) * error_measurement_gyro;


	//Initialising inital state
	const Eigen::VectorXd x0 = Eigen::VectorXd::Zero(9);
	const Eigen::Vector3d init_angles(x0(3), x0(4), x0(5));

	//Initialising Kalman Filter
	KalmanFilter kalman(A, B, C, Q, R);
	kalman.init(x0);


	//Subscribing & Update loop
	while(ros::ok()){

		//Defining matrices


		imu_sub = node.subscribe("/imu0", 1, kalman_updater);
		orientation_pub = node.advertise<geometry_msgs::Quaternion>("/current_orientation", 1);
		ros::spinOnce();
	}

	return 0;
}


void kalman_updater(const sensor_msgs::Imu::ConstPtr & imu_data){

	if (!kalman.isinit()){ROS_INFO("Please initialize kalman first");}

	//Time Interval
	if (counter == 0){ t_last = imu_data->header.stamp.toNSec();}
	t_current = imu_data->header.stamp.toNSec();
	dt = pow(t_current - t_last, -9.0);											//Transforming nanoseconds into seconds
	t_last = t_current;

	//Updating matrix C
	if (counter != 0){

	C << 0*I, 0*I, 0*I, 0*I, get_rotation_matrix(current_angles),
		0*I, 0*I, 0*I, I, 0*I;
	}

	//Getting imu-measurements
	current_measurements << imu_data->linear_acceleration.x,
							imu_data->linear_acceleration.y,
							imu_data->linear_acceleration.z + g;						//Compensating external acceleration 
																						//--> x_ is transformed, therefore g always in z-direction
							imu_data->angular_velocity.x,
							imu_data->angular_velocity.y,
							imu_data->angular_velocity.z;

	//Updating kalman filter
	kalman.update(dt, C, current_measurements);

	//Transforming into quaternions
	current_state = kalman.state();
	current_angles(0) = current_state(4);
	current_angles(1) = current_state(5);
	current_angles(2) = current_state(6);

	current_orientation = transform_quaternion_euler(current_angles);
	orientation_pub.publish(current_orientation);

	//Counterupdate
	counter++;
}




