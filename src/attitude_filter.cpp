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
ros::Publisher pose_pub;

void kalman_updater(const sensor_msgs::Imu::ConstPtr & imu_data);

//Definition of errors
const float error_state_euler_dot = 40;
const float error_state_linear_acceleration = 10;
const float error_measurement_linear_accelerometer = 1;
const float error_measurement_gyro = 0.001;

//Initialising inital state
const Eigen::VectorXd x0 = Eigen::VectorXd::Zero(9,9);

//Declaration Kalman Filter
KalmanFilter_Orientation kalman;

int main(int argc, char** argv){

	ros::init(argc, argv, "attitudefilter");
	ros::NodeHandle node;

	
	//Initialising Kalman Filter
	kalman.init_with_errors(x0, error_state_euler_dot, error_state_linear_acceleration,
                    error_measurement_gyro, error_measurement_linear_accelerometer);

	//Subscribing & Update loop
	while(ros::ok()){

		imu_sub = node.subscribe("/imu0", 1, kalman_updater);
		orientation_pub = node.advertise<geometry_msgs::Vector3>("current_euler", 1);
		pose_pub = node.advertise<geometry_msgs::Pose>("/current_pose", 1);

		ros::spinOnce();
	}

	return 0;
}


void kalman_updater(const sensor_msgs::Imu::ConstPtr & imu_data){

	//Updating kalman filter
	kalman.update(imu_data);

	//Publishing euler angles & pose
	orientation_pub.publish(kalman.get_angles());
	pose_pub.publish(kalman.get_pose());
}




