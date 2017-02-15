#include "arc_msgs/State.h"
#include "arc_tools/coordinate_transform.hpp"

#include "Eigen/Dense"
#include <fstream>
#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Float64.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "arc_state_estimation/car_model.hpp"

//Definition of constants.
float DISTANCE_WHEEL_AXES = 2.3; //[m]
float LENGTH_WHEEL_AXIS = 1.6; //[m]
int QUEUE_LENGTH = 10;  //Updating queue ~ frequency.
float CURRENT_ARRAY_SEARCHING_WIDTH = 4.0; //[m]
float CURRENT_ARRAY_MAXIMAL_WIDTH = 100.0; //[m]
std::string LAST_PATH_FILENAME = "last_path_file";
std::string CURRENT_PATH_FILENAME = "path_file";
std::string TEACH_REPEAT = "teach";
//TODO: Konstanten als argv oder in yaml file.
//Subcriber and publisher.
ros::Subscriber left_wheel_sub;
ros::Subscriber rov_sub;
ros::Subscriber orb_sub;
ros::Subscriber right_wheel_sub;
ros::Subscriber steering_sub;
ros::Publisher path_pub;
ros::Publisher rear_axle_pub;
ros::Publisher state_pub;
ros::Publisher velodyne_pub;
//Path and output file.
std::ofstream stream;
arc_msgs::State state;
int array_position;
bool stop = false;
nav_msgs::Path current_path;
//Mode: Teach (0) or Repeat (1).
bool mode;
//Declaration of functions.
double calculate_distance(geometry_msgs::Pose base, geometry_msgs::Pose target);
void close_state_estimation();
geometry_msgs::Pose getPose(nav_msgs::Path path, int index);
void init_state_estimation(ros::NodeHandle* node);
void odomUpdater();
void orbslam_sub(const nav_msgs::Odometry::ConstPtr & odom_data);
void rovio_sub(const nav_msgs::Odometry::ConstPtr & odom_data);
int search_current_array_position(const std::string teach_path_file); 
void steering_angle_sub(const std_msgs::Float64::ConstPtr& msg);
void tfBroadcaster(const Eigen::Vector4d euler, const Eigen::Vector3d position, std::string tf_name);
void velocity_left_sub(const std_msgs::Float64::ConstPtr& msg);
void velocity_right_sub(const std_msgs::Float64::ConstPtr& msg);
//Init class objects.
arc_state_estimation::CarModel car_model(DISTANCE_WHEEL_AXES, LENGTH_WHEEL_AXIS);

int main(int argc, char** argv){
	ros::init(argc, argv, "arc_state_estimation");
	ros::NodeHandle node;
  //Initialising.
  init_state_estimation(&node);
  //Spinning.
	ros::spin();
  //Closing state estimation.
  close_state_estimation();
	return 0;
}

void init_state_estimation(ros::NodeHandle* node){
  //Setting mode.
  if(TEACH_REPEAT == "teach") mode = false;
  else if(TEACH_REPEAT == "repeat") mode = true;
  else{
    std::cout << std::endl << "Please set mode: 'teach' or 'repeat'" << std::endl;
    ros::shutdown();
  }
  //Initialise output stream.
  std::string filename_all = CURRENT_PATH_FILENAME+".txt";
  stream.open(filename_all.c_str());
  //Initialising path_array.
  array_position = 0;
  // Publisher and subscriber.
  car_model.createPublisher(node);
  left_wheel_sub = node->subscribe("v_left", QUEUE_LENGTH, velocity_left_sub);
  right_wheel_sub = node->subscribe("v_right", QUEUE_LENGTH, velocity_right_sub);
  steering_sub = node->subscribe("steer_angle", QUEUE_LENGTH, steering_angle_sub);
  path_pub = node->advertise<nav_msgs::Path>("/path", QUEUE_LENGTH);
  rear_axle_pub = node->advertise<geometry_msgs::Transform>("/rear_axle/odom", QUEUE_LENGTH);
  state_pub = node->advertise<arc_msgs::State>("/state", QUEUE_LENGTH);
  velodyne_pub = node->advertise<geometry_msgs::Transform>("/velodyne/odom", QUEUE_LENGTH);
  //Mode dependent publisher and subscriber.
  rov_sub = node->subscribe("rovio/odometry", QUEUE_LENGTH, rovio_sub);
  orb_sub = node->subscribe("orb_slam2/odometry", QUEUE_LENGTH, orbslam_sub);
  std::cout << std::endl << "STATE ESTIMATION: Initialised, mode " << TEACH_REPEAT << std::endl;
}

void close_state_estimation(){
  std::cout << std::endl << "STATE ESTIMATION: Closing" << std::endl;
  //Closing stream.
  stream.close();
  //Finishing ros.
  ros::shutdown();
}


void rovio_sub(const nav_msgs::Odometry::ConstPtr & odom_data){
  //If teach the take position out of rovio, orientation and velocity always from rovio.
  if(!mode) state.pose.pose.position = odom_data->pose.pose.position;
  state.pose.pose.orientation = odom_data->pose.pose.orientation;
  state.pose_diff.twist = odom_data->twist.twist;
  //update state and path.
  odomUpdater();
}

void orbslam_sub(const nav_msgs::Odometry::ConstPtr & odom_data){
  //If repeat then take position from orbslam.
  if(mode) state.pose.pose.position = odom_data->pose.pose.position; 
  //TODO: Rovio set to pose in rovio.
  //Update state and path.
  odomUpdater();
}

void odomUpdater(){
  //Indexing state: if teach then iterate, if repeat search closest point.
  if(!mode) array_position += 1; 
  if(mode) array_position = search_current_array_position(LAST_PATH_FILENAME);
  //Updating state.
  state.stop = stop;
  state.current_arrayposition = array_position;
  current_path.poses.push_back(state.pose);
  //Tf broadcasting.
  Eigen::Vector3d position = arc_tools::transformPointMessageToEigen(state.pose.pose.position);
  Eigen::Vector4d quat = arc_tools::transformQuatMessageToEigen(state.pose.pose.orientation);
  Eigen::Vector3d lin_vel = arc_tools::transformVectorMessageToEigen(state.pose_diff.twist.linear);
  Eigen::Vector3d ang_vel = arc_tools::transformVectorMessageToEigen(state.pose_diff.twist.angular);
  tfBroadcaster(quat, position, "vi");
  //Updating model.
  car_model.updateModel(quat);
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

void steering_angle_sub(const std_msgs::Float64::ConstPtr& msg){
  float steering_angle = msg->data;
  car_model.set_steering_angle(steering_angle);
}

void velocity_left_sub(const std_msgs::Float64::ConstPtr& msg){
  float velocity_left = msg->data;
  car_model.set_velocity_left(velocity_left);
}

void velocity_right_sub(const std_msgs::Float64::ConstPtr& msg){
  float velocity_right = msg->data;
  car_model.set_velocity_right(velocity_right);
}

void tfBroadcaster(Eigen::Vector4d quat, Eigen::Vector3d position, std::string tf_name){
  // Init static broadcaster.
  static tf::TransformBroadcaster broadcaster;
  //Set orientation and vector.
  tf::Quaternion tf_quat(quat(0), quat(1), quat(2), quat(3));
  tf::Vector3 tf_vector(position(0), position(1), position(2)+0.2);
  //Setting tf - broadcast from odom to rear_axle.
  broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf_quat, tf_vector),
                           ros::Time::now(), "odom", tf_name));
}

int search_current_array_position(const std::string teach_path_file){
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
  while(!stream.eof()&& i<length){
    geometry_msgs::PoseStamped temp_pose;
    stream>>array_index;
    stream>>temp_pose.pose.position.x;
    stream>>temp_pose.pose.position.y;
    stream>>temp_pose.pose.position.z; 
    teach_path.poses.push_back(temp_pose);
    stream.ignore (300, '|');
    i++;  
  }
  //Finding last array position and current state.
  int last_array_position = array_position;
  geometry_msgs::Pose pose = state.pose.pose;
  //Finding maximal index so that in maximal width.
  int max_index = 0;
  while(calculate_distance(pose, teach_path.poses[last_array_position+max_index].pose)
                 < CURRENT_ARRAY_SEARCHING_WIDTH){
    max_index += 1;
  }
  //Searching closest point.
  double shortest_distance = CURRENT_ARRAY_MAXIMAL_WIDTH;
  int smallest_distance_index = last_array_position;
  for (int s = last_array_position-max_index; s < last_array_position+max_index; s++){
    double current_distance = calculate_distance(pose, teach_path.poses[s].pose);
    if (current_distance < shortest_distance){
      shortest_distance = current_distance;
      smallest_distance_index = s;
    }
  }
  //Check maximal distance.
  if (shortest_distance == CURRENT_ARRAY_MAXIMAL_WIDTH) state.stop = true; 
  return smallest_distance_index;
}

double calculate_distance(geometry_msgs::Pose base, geometry_msgs::Pose target){
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

geometry_msgs::Pose getPose(nav_msgs::Path path, int index){
  geometry_msgs::Pose pose;
  pose.position.x = path.poses[index].pose.position.x;
  pose.position.y = path.poses[index].pose.position.y;
  pose.position.z = path.poses[index].pose.position.z;
  return pose;
}

