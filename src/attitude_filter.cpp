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
const float error_state_euler_dot = 40 * 0.0001;
const float error_state_linear_acceleration = 10 * 0.001;
const float error_measurement_linear_accelerometer = 1 * 0.001;
const float error_measurement_gyro = 0.001 * 0.001;

//Updating Queue ~ Updating Frequency
int queue_length = 200;


//Declaration Kalman Filter
KalmanFilter_Orientation kalman;

int main(int argc, char** argv){

	ros::init(argc, argv, "attitudefilter");
	ros::NodeHandle node;

	//Initialising inital state
	Eigen::VectorXd x_0(15);
	x_0[0]= 10.0; x_0[1]=0.0; x_0[2]=0.0;	//positions
	x_0[3]= 0.0; x_0[4]=0.0; x_0[5]=0.0;	//euler angles
	x_0[6]= 0.0; x_0[7]=0.0; x_0[8]=0.0;	//linear velocities
	x_0[9]= 0.0; x_0[10]=0.0; x_0[11]=0.0;	//angular velocities
	x_0[12]= 0.0; x_0[13]=0.0; x_0[14]=0.0;	//linear accelerations
	
	//Initialising Kalman Filter
	kalman.init_with_errors(x_0, 
					error_state_euler_dot, error_state_linear_acceleration,
                    error_measurement_gyro, error_measurement_linear_accelerometer);

	//Subscribing & Update loop
	while(ros::ok()){

		imu_sub = node.subscribe("/imu0", queue_length, kalman_updater);
		orientation_pub = node.advertise<geometry_msgs::Vector3>("current_euler", queue_length);
		pose_pub = node.advertise<geometry_msgs::Pose>("/current_pose", queue_length);

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




