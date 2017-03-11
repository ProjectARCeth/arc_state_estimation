#include "arc_msgs/State.h"
#include "arc_tools/coordinate_transform.hpp"
#include "arc_tools/tf_kit.hpp"

#include "Eigen/Dense"
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"

#include "arc_state_estimation/car_model.hpp"

//Definition of constants.
float CAM_INIT_QUAT_X;
float CAM_INIT_QUAT_Y;
float CAM_INIT_QUAT_W;
float CAM_INIT_QUAT_Z;
float DISTANCE_WHEEL_AXES;
float LENGTH_WHEEL_AXIS;
int QUEUE_LENGTH;
float CURRENT_ARRAY_SEARCHING_WIDTH;
float MAX_DEVIATION_FROM_TEACH_PATH;
float MAX_VELOCITY_DIVERGENCE;
float MAX_ABSOLUTE_VELOCITY;
float MAX_ORIENTATION_DIVERGENCE;
float MIN_SHUTDOWN_VELOCITY;
std::string CURRENT_PATH_FILENAME;
std::string LAST_PATH_FILENAME;
std::string MODE_TOPIC;
std::string PATH_TOPIC;
std::string SHUTDOWN_TOPIC;
std::string STATE_TOPIC;
std::string STEERING_ANGLE_TOPIC;
std::string TEACH_PATH_TOPIC;
std::string TRACKING_ERROR_TOPIC;
std::string TRACKING_VEL_ERROR_TOPIC;
std::string WHEEL_SENSORS_LEFT_TOPIC;
std::string WHEEL_SENSORS_RIGHT_TOPIC;
//Subcriber and publisher.
ros::Subscriber left_wheel_sub;
ros::Subscriber mode_sub;
ros::Subscriber rov_sub;
ros::Subscriber orb_sub;
ros::Subscriber right_wheel_sub;
ros::Subscriber steering_sub;
ros::Subscriber shutdown_sub;
ros::Subscriber stop_sub;
ros::Publisher path_pub;
ros::Publisher state_pub;
ros::Publisher path_teach_pub;
ros::Publisher tracking_error_pub;
ros::Publisher tracking_error_vel_pub;
//Path and output file.
std::ofstream stream;
arc_msgs::State state;
int array_position;
bool stop = false;
bool shutdown = false;
nav_msgs::Path current_path;
//Mode: Teach (0) = Default or Repeat (1).
bool mode = false; 
//State as an Eigen::Vector.
Eigen::Vector3d position;
Eigen::Vector4d quat;
Eigen::Vector3d lin_vel;
Eigen::Vector3d ang_vel;
//Declaration of functions.
double calculateDistance(geometry_msgs::Pose base, geometry_msgs::Pose target);
void closeStateEstimation();
void initStateEstimation(ros::NodeHandle* node);
void modeCallback(const std_msgs::Bool::ConstPtr& msg);
void odomUpdater();
void orbslamCallback(const nav_msgs::Odometry::ConstPtr& odom_data);
void rovioCallback(const nav_msgs::Odometry::ConstPtr& odom_data);
int searchCurrentArrayPosition(const std::string teach_path_file); 
void shutdownCallback(const std_msgs::Bool::ConstPtr& msg);
void steeringAngleCallback(const std_msgs::Float64::ConstPtr& msg);
void stopWithReason(std::string reason);
void velocityLeftCallback(const std_msgs::Float64::ConstPtr& msg);
void velocityRightCallback(const std_msgs::Float64::ConstPtr& msg);
//Init class objects.
arc_state_estimation::CarModel car_model(DISTANCE_WHEEL_AXES, LENGTH_WHEEL_AXIS);

int main(int argc, char** argv){
  //Init ROS.
	ros::init(argc, argv, "arc_state_estimation");
	ros::NodeHandle node;
  //Getting parameter.
  node.getParam("/sensor/CAM_INIT_QUAT_X", CAM_INIT_QUAT_X);
  node.getParam("/sensor/CAM_INIT_QUAT_Y", CAM_INIT_QUAT_Y);
  node.getParam("/sensor/CAM_INIT_QUAT_Z", CAM_INIT_QUAT_Z);
  node.getParam("/sensor/CAM_INIT_QUAT_W", CAM_INIT_QUAT_W);
  node.getParam("/erod/DISTANCE_WHEEL_AXES", DISTANCE_WHEEL_AXES);
  node.getParam("/erod/LENGTH_WHEEL_AXIS", LENGTH_WHEEL_AXIS);
  node.getParam("/general/QUEUE_LENGTH", QUEUE_LENGTH);
  node.getParam("/general/CURRENT_ARRAY_SEARCHING_WIDTH", CURRENT_ARRAY_SEARCHING_WIDTH);
  node.getParam("/safety/MAX_DEVIATION_FROM_TEACH_PATH", MAX_DEVIATION_FROM_TEACH_PATH);
  node.getParam("/safety/MAX_ORIENTATION_DIVERGENCE", MAX_ORIENTATION_DIVERGENCE);
  node.getParam("/safety/MIN_SHUTDOWN_VELOCITY", MIN_SHUTDOWN_VELOCITY);
  node.getParam("/files/LAST_PATH_FILENAME", LAST_PATH_FILENAME);
  node.getParam("/files/CURRENT_PATH_FILENAME", CURRENT_PATH_FILENAME);
  node.getParam("/topic/STATE", STATE_TOPIC);
  node.getParam("/topic/STEERING_ANGLE", STEERING_ANGLE_TOPIC);
  node.getParam("/topic/WHEEL_REAR_LEFT", WHEEL_SENSORS_LEFT_TOPIC);
  node.getParam("/topic/WHEEL_REAR_RIGHT", WHEEL_SENSORS_RIGHT_TOPIC);
  node.getParam("/topic/TRACKING_ERROR", TRACKING_ERROR_TOPIC);
  node.getParam("/topic/TRACKING_ERROR_VELOCITY", TRACKING_VEL_ERROR_TOPIC);
  node.getParam("/topic/MODE", MODE_TOPIC);
  node.getParam("/topic/PATH", PATH_TOPIC);
  node.getParam("/topic/TEACH_PATH", TEACH_PATH_TOPIC);
  node.getParam("/topic/SHUTDOWN", SHUTDOWN_TOPIC);
  // Initialising.
  initStateEstimation(&node);
  //Spinning.
	ros::spin();
	return 0;
}

void initStateEstimation(ros::NodeHandle* node){
  //Initialise output stream.
  std::string filename_all = CURRENT_PATH_FILENAME+".txt";
  stream.open(filename_all.c_str());
  //Initialising path_array.
  array_position = 0;
  // Publisher and subscriber.
  car_model.createPublisher(node);
  left_wheel_sub = node->subscribe(WHEEL_SENSORS_LEFT_TOPIC, QUEUE_LENGTH, velocityLeftCallback);
  mode_sub = node->subscribe(MODE_TOPIC, QUEUE_LENGTH, modeCallback);
  orb_sub = node->subscribe("orb_slam2/odometry", QUEUE_LENGTH, orbslamCallback);
  right_wheel_sub = node->subscribe(WHEEL_SENSORS_RIGHT_TOPIC, QUEUE_LENGTH, velocityRightCallback);
  rov_sub = node->subscribe("rovio/odometry", QUEUE_LENGTH, rovioCallback);
  shutdown_sub = node->subscribe(SHUTDOWN_TOPIC, QUEUE_LENGTH, shutdownCallback);
  steering_sub = node->subscribe(STEERING_ANGLE_TOPIC, QUEUE_LENGTH, steeringAngleCallback);
  path_pub = node->advertise<nav_msgs::Path>(PATH_TOPIC, QUEUE_LENGTH);
  path_teach_pub = node->advertise<nav_msgs::Path>(TEACH_PATH_TOPIC, QUEUE_LENGTH);
  state_pub = node->advertise<arc_msgs::State>(STATE_TOPIC, QUEUE_LENGTH);
  tracking_error_pub = node->advertise<std_msgs::Float64>(TRACKING_ERROR_TOPIC, QUEUE_LENGTH);
  tracking_error_vel_pub = node->advertise<std_msgs::Float64>(TRACKING_VEL_ERROR_TOPIC, QUEUE_LENGTH);
  std::cout << std::endl << "STATE ESTIMATION: Initialised" << std::endl;  
}

void closeStateEstimation(){
  std::cout << std::endl << "STATE ESTIMATION: Closing" << std::endl;
  //Closing stream.
  stream.close();
  //Finishing ros.
  ros::shutdown();
  ros::waitForShutdown();
}

void rovioCallback(const nav_msgs::Odometry::ConstPtr & odom_data){
  //Velocity out of Rovio.
  state.pose_diff.twist = odom_data->twist.twist;
  //Update state and path.
  odomUpdater();
}

void orbslamCallback(const nav_msgs::Odometry::ConstPtr & odom_data){
  //Position (global) out of orbslam.
  state.pose.pose.position = odom_data->pose.pose.position;
  //Orientation out of orbslam.
  //only relative rotation, e.g. substract orient).
  Eigen::Vector4d quat = arc_tools::transformQuatMessageToEigen(odom_data->pose.pose.orientation);
  Eigen::Vector4d quat_init_soll(0,0,0,1);
  Eigen::Vector4d quat_init(CAM_INIT_QUAT_X, CAM_INIT_QUAT_Y, CAM_INIT_QUAT_Z, CAM_INIT_QUAT_W);
  Eigen::Vector4d quat_init_trafo = arc_tools::diffQuaternion(quat_init, quat_init_soll);
  Eigen::Vector4d quat_diff = arc_tools::diffQuaternion(quat_init_trafo, quat); 
  state.pose.pose.orientation =  arc_tools::transformEigenToQuatMessage(quat_diff);
  //Update state and path.
  odomUpdater();
}

void odomUpdater(){
  //Tf broadcasting.
  position = arc_tools::transformPointMessageToEigen(state.pose.pose.position);
  quat = arc_tools::transformQuatMessageToEigen(state.pose.pose.orientation);
  lin_vel = arc_tools::transformVectorMessageToEigen(state.pose_diff.twist.linear);
  ang_vel = arc_tools::transformVectorMessageToEigen(state.pose_diff.twist.angular);
  arc_tools::tfBroadcaster(quat, position, "odom", "vi");
  //Indexing state: if teach then iterate, if repeat search closest point.
  if(!mode) array_position += 1; 
  if(mode) array_position = searchCurrentArrayPosition(LAST_PATH_FILENAME);
  //Updating state.
  state.stop = stop;
  state.current_arrayposition = array_position;
  current_path.poses.push_back(state.pose);
  //Publishing.
  state_pub.publish(state);
  path_pub.publish(current_path);
  //Writing path File.
  stream <<array_position<<" "<<
           position(0)<<" "<<position(1)<<" "<<position(2)<<" "<<
           quat(0)<<" "<<quat(1)<<" "<<quat(2)<<" "<<quat(3)<<" "<<
           lin_vel(0)<<" "<<lin_vel(1)<<" "<<lin_vel(2)<<" "<<
           ang_vel(0)<<" "<<ang_vel(1)<<" "<<ang_vel(2)<<" "<<stop<<"|";
} 

void modeCallback(const std_msgs::Bool::ConstPtr& msg){
  mode = msg->data;
}

void shutdownCallback(const std_msgs::Bool::ConstPtr& msg){
  shutdown = msg->data;
  // Shutdown iff velocity is close to zero (control needs state estimation).
  if (shutdown){
    if (lin_vel.norm() < MIN_SHUTDOWN_VELOCITY) closeStateEstimation();
  }
}

void steeringAngleCallback(const std_msgs::Float64::ConstPtr& msg){
  float steering_angle = msg->data;
  car_model.set_steering_angle(steering_angle);
  //Updating model.
  car_model.updateModel(quat);
}

void velocityLeftCallback(const std_msgs::Float64::ConstPtr& msg){
  float velocity_left = msg->data;
  car_model.set_velocity_left(velocity_left);
  //Updating model.
  car_model.updateModel(quat);
}

void velocityRightCallback(const std_msgs::Float64::ConstPtr& msg){
  float velocity_right = msg->data;
  car_model.set_velocity_right(velocity_right);
  //Updating model.
  car_model.updateModel(quat);
}

int searchCurrentArrayPosition(const std::string teach_path_file){
  //Read in teach_path.
  //Open stream.
  std::fstream fin;
  fin.open(teach_path_file.c_str());
  if(!fin.is_open()){
    std::cout<<std::endl<<"STATE ESTIMATION: Error with opening of  "
             <<teach_path_file<<std::endl;
  }
  //Getting file length.
  fin.seekg (-2, fin.end); 
  int length = fin.tellg();
  fin.seekg (0, fin.beg);
  //Copy file.
  char * file = new char [length];
  fin.read (file,length);
  std::istringstream stream(file,std::ios::in);
  delete[] file;  
  fin.close () ;
  //Writing path from file.
  int i=0;
  int array_index;  
  nav_msgs::Path teach_path;
  nav_msgs::Path teach_path_diff;
  while(!stream.eof()&& i<length){
    geometry_msgs::PoseStamped temp_pose;
    geometry_msgs::PoseStamped temp_pose_diff;
    stream>>array_index;
    stream>>temp_pose.pose.position.x;
    stream>>temp_pose.pose.position.y;
    stream>>temp_pose.pose.position.z;
    //Reading teach orientation
    stream>>temp_pose.pose.orientation.x;
    stream>>temp_pose.pose.orientation.y;
    stream>>temp_pose.pose.orientation.z;
    stream>>temp_pose.pose.orientation.w;
    //Reading teach velocity
    stream>>temp_pose_diff.pose.position.x;
    stream>>temp_pose_diff.pose.position.y;
    stream>>temp_pose_diff.pose.position.z;
    teach_path.poses.push_back(temp_pose);
    teach_path_diff.poses.push_back(temp_pose_diff);
    stream.ignore (300, '|');
    i++;  
  }
  //Publish teach path for visualization.
  path_teach_pub.publish(teach_path);
  //Finding last array position and current state.
  int last_array_position = array_position;
  geometry_msgs::Pose last_pose = teach_path.poses[last_array_position].pose;
  geometry_msgs::Pose pose = state.pose.pose;
  //Finding maximal index so that in maximal width.
  int max_index = 0;
  while(calculateDistance(last_pose, teach_path.poses[last_array_position+max_index].pose)
                 < CURRENT_ARRAY_SEARCHING_WIDTH){
    max_index += 1;
  }
  //Searching closest point.
  double shortest_distance = MAX_DEVIATION_FROM_TEACH_PATH;
  int smallest_distance_index = last_array_position;
  for (int s = last_array_position-max_index; s < last_array_position+max_index; s++){
    double current_distance = calculateDistance(pose, teach_path.poses[s].pose);
    if (current_distance < shortest_distance){
      shortest_distance = current_distance;
      smallest_distance_index = s;
    }
  }
  //Checkliste safety.
  //1)Check maximal distance.
  if (shortest_distance >= MAX_DEVIATION_FROM_TEACH_PATH) stopWithReason("deviation from teach path");
  //2)Check absolute maximal velocity (only xy direction).
  float v_abs=sqrt(pow(state.pose_diff.twist.linear.x,2)+pow(state.pose_diff.twist.linear.y,2));
  if (v_abs >= MAX_ABSOLUTE_VELOCITY) stopWithReason("absolute velocity");
  //3)Check divergence to teach velocity (only xy direction).
  float v_teach=sqrt(pow(teach_path_diff.poses[smallest_distance_index].pose.position.x,2)+pow(teach_path_diff.poses[smallest_distance_index].pose.position.y,2));
  if ((v_abs-v_teach)>=MAX_VELOCITY_DIVERGENCE) stopWithReason("velocity divergence");
  //4)Check divergence to teach orientation (normal to plane orientation).
  float current_orientation = arc_tools::transformEulerQuaternionVector(quat)(2);
  Eigen::Vector4d path_quat = arc_tools::transformQuatMessageToEigen(teach_path.poses[smallest_distance_index].pose.orientation);
  float path_orientation = arc_tools::transformEulerQuaternionVector(path_quat)(2);
  float alpha=abs(current_orientation - path_orientation);
  if (alpha>=MAX_ORIENTATION_DIVERGENCE) stopWithReason("orientation divergence");
  //Publish tracking error.
  std_msgs::Float64 shortest_distance_msg;
  shortest_distance_msg.data = shortest_distance;
  tracking_error_pub.publish(shortest_distance_msg);
  //Publish tracking error velocity.
  std_msgs::Float64 velocity_tracking_error_msg;
  geometry_msgs::Point temp_vel = teach_path_diff.poses[smallest_distance_index].pose.position;
  Eigen::Vector3d path_vel(temp_vel.x, temp_vel.y, temp_vel.z);
  velocity_tracking_error_msg.data = (path_vel - lin_vel).norm();
  tracking_error_vel_pub.publish(velocity_tracking_error_msg);
  return smallest_distance_index;
}

double calculateDistance(geometry_msgs::Pose base, geometry_msgs::Pose target){
  double x_base = base.position.x;
  double y_base = base.position.y;
  double z_base = base.position.z;
  double x_target = target.position.x;
  double y_target = target.position.y;
  double z_target = target.position.z;
  double distance = sqrt((x_base-x_target)*(x_base-x_target) + 
                        (y_base-y_target)*(y_base-y_target) + (z_base-z_target)*(z_base-z_target));
  return distance;
}

void stopWithReason(std::string reason){
  stop = true;
  std::cout << std::endl << "STATE ESTIMATION: STOP due to " << reason << std::endl;
}