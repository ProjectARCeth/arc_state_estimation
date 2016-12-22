#include "arc_tools/coordinate_transform.hpp"
#include "arc_tools/state_and_path_publisher.hpp"

#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Transform.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

ros::Subscriber orb_sub;
ros::Publisher velodyne_pub;
ros::Publisher rear_axle_pub;
//Declaration of functions.
void odomUpdaterOrb(const nav_msgs::Odometry::ConstPtr & odom_data);
void tfBroadcaster(const Eigen::Vector4d euler, const Eigen::Vector3d position, std::string tf_name);
void tfListener(std::string tf_name);
//Updating Queue ~ Updating Frequency.
int queue_length = 10;
//Init class objects of Publisher and Filter.
arc_tools::StateAndPathPublisher pub_state("path", "path");

int main(int argc, char** argv){
	ros::init(argc, argv, "arc_state_estimation");
	ros::NodeHandle node;
  //Initialising publisher.
	pub_state.createPublisher(&node);
	//Subscribing & Update.
  orb_sub = node.subscribe("orb_slam2/odometry", queue_length, odomUpdaterOrb);
  rear_axle_pub = node.advertise<geometry_msgs::Transform>("/rear_axle/odom", queue_length);
  velodyne_pub = node.advertise<geometry_msgs::Transform>("/velodyne/odom", queue_length);
	ros::spin();
	return 0;
}

void odomUpdaterOrb(const nav_msgs::Odometry::ConstPtr & odom_data){
  //Publishing euler angles & pose.
  Eigen::Vector3d position = arc_tools::transformPointMessageToEigen(odom_data->pose.pose.position);
  Eigen::Vector4d quat = arc_tools::transformQuatMessageToEigen(odom_data->pose.pose.orientation);
  Eigen::Vector3d lin_vel = arc_tools::transformVectorMessageToEigen(odom_data->twist.twist.linear);
  Eigen::Vector3d ang_vel = arc_tools::transformVectorMessageToEigen(odom_data->twist.twist.angular);
  //Publishing.
  pub_state.publishWithQuaternion(position, quat, lin_vel, ang_vel, false);
  tfBroadcaster(quat, position, "vi");
  tfListener("velodyne");
  tfListener("rear_axle");
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

void tfListener(std::string tf_name){
  //Init static listener.
  static tf::TransformListener listener;
  //Listening.
  tf::StampedTransform tf_transform;
  try{listener.lookupTransform("odom", tf_name, ros::Time(0), tf_transform);}
  catch(tf::TransformException &ex){}
  //Publishing transformation.
  geometry_msgs::Transform transform;
  transform.translation.x = tf_transform.getOrigin().x();
  transform.translation.y = tf_transform.getOrigin().y();
  transform.translation.z = tf_transform.getOrigin().z();
  transform.rotation.x = tf_transform.getRotation().x();
  transform.rotation.y = tf_transform.getRotation().y();
  transform.rotation.z = tf_transform.getRotation().z();
  transform.rotation.w = tf_transform.getRotation().w();
  velodyne_pub.publish(transform);
}