#include "arc_header/coordinate_transform.h"
#include "arc_header/kalman_filter.h"

#include <cmath>
#include <eigen3/Eigen/Eigen>

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"


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

const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
Eigen::MatrixXd A;
const Eigen::MatrixXd B = Eigen::MatrixXd::Zero(15, 15);
Eigen::MatrixXd C;

//Initialising inital state
const Eigen::VectorXd x0 = Eigen::VectorXd::Zero(9);
const Eigen::Vector3d init_angles(x0(3), x0(4), x0(5));

//Initialising error matrices
Eigen::MatrixXd R;
Eigen::VectorXd Q;

//Initialization for kalman Update (global: no new initialization every step)
Eigen::Vector3d current_angles;
Eigen::VectorXd current_state;
Eigen::VectorXd current_measurements;
geometry_msgs::Quaternion current_orientation;

//Initialising Kalman Filter
KalmanFilter kalman(A, B, C, Q, R);

int main(int argc, char** argv){

	ros::init(argc, argv, "attitudefilter");
	ros::NodeHandle node;


	//Defining state matrices
	A << I, 0*I, dt*I, 0*I, 0*I,
		0*I, I, 0*I, dt*I, 0*I,
		0*I, 0*I, I, 0*I, dt*I,
		0*I, 0*I, 0*I, I, 0*I,
		0*I, 0*I, 0*I, 0*I, 0*I;
	C << 0*I, 0*I, 0*I, 0*I, get_rotation_matrix(init_angles),
		0*I, 0*I, 0*I, I, 0*I;
	R << I,0*I,0*I,0*I,0*I,
		0*I,I,0*I,0*I,0*I,
		0*I,0*I,I,0*I,0*I,
		0*I,0*I,0*I,error_state_euler_dot*I,0*I,
		0*I,0*I,0*I,0*I,error_state_linear_acceleration*I;
	Q << error_measurement_linear_accelerometer,
		error_measurement_linear_accelerometer,
		error_measurement_linear_accelerometer, 
		error_measurement_gyro, 
		error_measurement_gyro, 
		error_measurement_gyro;

	//Initialising Kalman Filter
	kalman.init(x0);

	//Subscribing & Update loop
	while(ros::ok()){

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




