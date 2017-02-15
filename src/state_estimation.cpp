#include "arc_tools/coordinate_transform.hpp"
#include "arc_state_estimation/car_model.hpp"
#include "arc_state_estimation/state_and_path_publisher.hpp"

#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Transform.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//Definition of constants.
float DISTANCE_WHEEL_AXES = 2.3;
float LENGTH_WHEEL_AXIS = 1.6; 

ros::Subscriber steering_sub;
ros::Subscriber left_wheel_sub;
ros::Subscriber right_wheel_sub;
ros::Subscriber orb_sub;
ros::Publisher velodyne_pub;
ros::Publisher rear_axle_pub;
//Declaration of functions.
void steering_angle_sub(const std_msgs::Float64::ConstPtr& msg);
void velocity_left_sub(const std_msgs::Float64::ConstPtr& msg);
void velocity_right_sub(const std_msgs::Float64::ConstPtr& msg);
void odomUpdater(const nav_msgs::Odometry::ConstPtr & odom_data);
void tfBroadcaster(const Eigen::Vector4d euler, const Eigen::Vector3d position, std::string tf_name);
//Updating Queue ~ Updating Frequency.
int queue_length = 10;
//Init class objects.
arc_state_estimation::StateAndPathPublisher pub_state("path", "path");
arc_state_estimation::CarModel car_model(DISTANCE_WHEEL_AXES, LENGTH_WHEEL_AXIS);

int main(int argc, char** argv){
	ros::init(argc, argv, "arc_state_estimation");
	ros::NodeHandle node;
  //Initialising publisher.
	pub_state.createPublisher(&node);
  car_model.createPublisher(&node);
	//Subscribing & Update.
  steering_sub = node.subscribe("steer_angle", queue_length, steering_angle_sub);
  left_wheel_sub = node.subscribe("v_left", queue_length, velocity_left_sub);
  right_wheel_sub = node.subscribe("v_right", queue_length, velocity_right_sub);
  orb_sub = node.subscribe("rovio/odometry", queue_length, odomUpdater);
  rear_axle_pub = node.advertise<geometry_msgs::Transform>("/rear_axle/odom", queue_length);
  velodyne_pub = node.advertise<geometry_msgs::Transform>("/velodyne/odom", queue_length);
	ros::spin();
	return 0;
}

void odomUpdater(const nav_msgs::Odometry::ConstPtr & odom_data){
  //Publishing euler angles & pose.
  Eigen::Vector3d position = arc_tools::transformPointMessageToEigen(odom_data->pose.pose.position);
  Eigen::Vector4d quat = arc_tools::transformQuatMessageToEigen(odom_data->pose.pose.orientation);
  Eigen::Vector3d lin_vel = arc_tools::transformVectorMessageToEigen(odom_data->twist.twist.linear);
  Eigen::Vector3d ang_vel = arc_tools::transformVectorMessageToEigen(odom_data->twist.twist.angular);
  //Updating model.
  car_model.updateModel(quat);
  //Publishing.
  pub_state.publishWithQuaternion(position, quat, lin_vel, ang_vel, false);
  tfBroadcaster(quat, position, "vi");
} 

void tfBroadcaster(Eigen::Vector4d quat, Eigen::Vector3d position, std::string tf_name){
  // Init static broadcaster.
  static tf::TransformBroadcaster broadcaster;
  //Set orientation and vector.
  tf::Quaternion tf_quat(quat(0), quat(1), quat(2), quat(3));
  tf::Vector3 tf_vector(position(0), position(1), position(2)+0.2);
  //Setting tf - broadcast from odom to rear_axle.
  broadcaster.sendTransform(
      tf::StampedTransform(tf::Transform(tf_quat, tf_vector),
                           ros::Time::now(), "odom", tf_name));
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