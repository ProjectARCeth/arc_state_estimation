#include "arc_header/coordinate_transform.h"
#include "arc_header/kalman_filter.h"

#include <cmath>
#include <eigen3/Eigen/Eigen>

#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Imu.h"

ros::Subscriber imuSub;
ros::Publisher orientationPub;
ros::Publisher posePub;
//Declaration of functions
void kalmanUpdater(const sensor_msgs::Imu::ConstPtr & imu_data);
void getParameters(ros::NodeHandle& node);
//Definition of errors
double errorStateEulerDot;
double errorStateLinearAcceleration;
double errorMeasurementLinearAccelerometer;
double errorMeasurementGyro;
//Updating Queue ~ Updating Frequency
int queueLength;
//Declaration Kalman Filter
KalmanFilterOrientation kalman;

int main(int argc, char** argv){
	ros::init(argc, argv, "attitudefilter");
	ros::NodeHandle node;		
	//Getting parameters
	getParameters(node);
	//Initialising inital state
	Eigen::VectorXd x_0(15);
	x_0[0]= 10.0; x_0[1]=0.0; x_0[2]=0.0;	//positions
	x_0[3]= 0.0; x_0[4]=0.0; x_0[5]=0.0;	//euler angles
	x_0[6]= 0.0; x_0[7]=0.0; x_0[8]=0.0;	//linear velocities
	x_0[9]= 0.0; x_0[10]=0.0; x_0[11]=0.0;	//angular velocities
	x_0[12]= 0.0; x_0[13]=0.0; x_0[14]=0.0;	//linear accelerations
	//Initialising Kalman Filter
	kalman.initWithErrors(x_0, 
					errorStateEulerDot, errorStateLinearAcceleration,
                    errorMeasurementGyro, errorMeasurementLinearAccelerometer);
	//Subscribing & Update loop
	while(ros::ok()){
		imuSub = node.subscribe("/imu0", queueLength, kalmanUpdater);
		orientationPub = node.advertise<geometry_msgs::Vector3>("current_euler", queueLength);
		posePub = node.advertise<geometry_msgs::Pose>("/current_pose", queueLength);
		ros::spinOnce();
	}
	return 0;
}

void kalmanUpdater(const sensor_msgs::Imu::ConstPtr & imuData){
	//Updating kalman filter
	kalman.update(imuData);
	//Publishing euler angles & pose
	orientationPub.publish(kalman.getAngles());
	posePub.publish(kalman.getPose());
}

void getParameters(ros::NodeHandle& node){

	node.getParam("/StateEstimation/ImuFilter/errorStateEulerDot", errorStateEulerDot);
	node.getParam("/StateEstimation/ImuFilter/errorStateLinearAcceleration", 
		errorMeasurementLinearAccelerometer);
	node.getParam("/StateEstimation/ImuFilter/errorMeasurementGyro", errorMeasurementGyro);
	node.getParam("/StateEstimation/ImuFilter/errorMeasurementLinearAccelerometer", 
		errorMeasurementLinearAccelerometer);
	node.getParam("/StateEstimation/ImuFilter/subAndPubQueueLength", queueLength);
}

