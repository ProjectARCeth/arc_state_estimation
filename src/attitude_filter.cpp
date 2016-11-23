#include "arc_tools/kalman_filter.hpp"

#include "yaml-cpp/yaml.h"

ros::Subscriber imu_sub;
ros::Publisher orientation_pub;
ros::Publisher pose_pub;
//Declaration of functions.
void kalmanUpdater(const sensor_msgs::Imu::ConstPtr & imu_data);
void getParameters(ros::NodeHandle* node);
//Definition of errors
double error_state_euler_dot;
double error_state_linear_acceleration;
double error_measurement_linear_accelerometer;
double error_measurement_gyro;
//Updating Queue ~ Updating Frequency.
int queue_length;
//Declaration Kalman Filter.
KalmanFilterOrientation kalman;

int main(int argc, char** argv){
	ros::init(argc, argv, "attitudefilter");
	ros::NodeHandle node;		
	//Getting parameters.
	getParameters(&node);
	//Initialising inital state.
	Eigen::VectorXd x_0(15);
	x_0[0]= 0.0; x_0[1]=0.0; x_0[2]=0.0;	//Positions.
	x_0[3]= 0.0; x_0[4]=0.0; x_0[5]=0.0;	//Euler angles.
	x_0[6]= 0.0; x_0[7]=0.0; x_0[8]=0.0;	//Linear velocities.
	x_0[9]= 0.0; x_0[10]=0.0; x_0[11]=0.0;	//Angular velocities.
	x_0[12]= 0.0; x_0[13]=0.0; x_0[14]=0.0;	//Linear accelerations.
	//Initialising Kalman Filter.
	kalman.initWithErrors(x_0, 
					error_state_euler_dot, error_state_linear_acceleration,
                    error_measurement_gyro, error_measurement_linear_accelerometer);
	//Subscribing & Update.
	imu_sub = node.subscribe("/imu0", queue_length, kalmanUpdater);
	orientation_pub = node.advertise<geometry_msgs::Vector3>("/current_euler", queue_length);
	pose_pub = node.advertise<geometry_msgs::Pose>("/current_pose", queue_length);
	ros::spin();
	return 0;
}

void kalmanUpdater(const sensor_msgs::Imu::ConstPtr & imuData){
	//Updating kalman filter.
	kalman.update(imuData);
	//Publishing euler angles & pose.
	orientation_pub.publish(kalman.getAngles());
	pose_pub.publish(kalman.getPose());
}

void getParameters(ros::NodeHandle* node){
	node->getParam("/StateEstimation/ImuFilter/errorStateEulerDot", error_state_euler_dot);
	node->getParam("/StateEstimation/ImuFilter/errorStateLinearAcceleration", 
		error_state_linear_acceleration);
	node->getParam("/StateEstimation/ImuFilter/errorMeasurementGyro", error_measurement_gyro);
	node->getParam("/StateEstimation/ImuFilter/errorMeasurementLinearAccelerometer", 
		error_measurement_linear_accelerometer);
	node->getParam("/StateEstimation/ImuFilter/subAndPubQueueLength", queue_length);
}
