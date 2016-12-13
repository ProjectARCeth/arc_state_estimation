#include "arc_tools/coordinate_transform.hpp"
#include "arc_tools/state_and_path_publisher.hpp"

#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>

ros::Subscriber rovio_sub;
ros::Subscriber orb_sub;
//Declaration of functions.
void odomUpdater(const nav_msgs::Odometry::ConstPtr & odom_data);
void tfBroadcaster(const Eigen::Vector4d euler, const Eigen::Vector3d position);
//Updating Queue ~ Updating Frequency.
int queue_length = 10;
//Init class objects of Publisher and Filter.
arc_tools::StateAndPathPublisher pub;
//Teststate.
Eigen::VectorXd test_state(12);

int main(int argc, char** argv){
	ros::init(argc, argv, "arc_state_estimation");
	ros::NodeHandle node;
  //Initialising publisher.
	pub.createPublisher(&node);
	//Subscribing & Update.
	rovio_sub = node.subscribe("rovio/odometry", queue_length, odomUpdater);
  orb_sub = node.subscribe("orb_slam2/odometry", queue_length, odomUpdater);
	ros::spin();
	return 0;
}

void odomUpdater(const nav_msgs::Odometry::ConstPtr & odom_data){
	//Publishing euler angles & pose.
  Eigen::Vector3d position = arc_tools::transformPointMessageToEigen(odom_data->pose.pose.position);
  Eigen::Vector4d quat = arc_tools::transformQuatMessageToEigen(odom_data->pose.pose.orientation);
  Eigen::Vector3d lin_vel = arc_tools::transformVectorMessageToEigen(odom_data->twist.twist.linear);
  Eigen::Vector3d ang_vel = arc_tools::transformVectorMessageToEigen(odom_data->twist.twist.angular);
  //Publishing.
	pub.publishWithQuaternion(position, quat, lin_vel, ang_vel, false);
	tfBroadcaster(quat, position);
}	

void tfBroadcaster(Eigen::Vector4d quat, Eigen::Vector3d position){
  // Init static broadcaster.
  static tf::TransformBroadcaster broadcaster;
  //Set orientation and vector.
  tf::Quaternion tf_quat(quat(0), quat(1), quat(2), quat(3));
  tf::Vector3 tf_vector(position(0), position(1), position(2));
  //Setting tf - broadcast from odom to rear_axle.
  broadcaster.sendTransform(
      tf::StampedTransform(tf::Transform(tf_quat, tf_vector),
                           ros::Time::now(), "odom", "rear_axle"));
}
