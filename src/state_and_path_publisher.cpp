#include "arc_state_estimation/state_and_path_publisher.hpp"

namespace arc_state_estimation{

StateAndPathPublisher::StateAndPathPublisher(std::string pub_name, std::string filename){
  //Setting Publisher name.
  pub_name_ = pub_name;
  //Initialising path txt_file.
  std::string filename_all = filename+".txt";
  stream_.open(filename_all.c_str());
  //Initialising path_array.
  array_position_ = 0;
  path_vector_.clear();
  path_.header.frame_id = pub_name_;

}

void StateAndPathPublisher::createPublisher(ros::NodeHandle* node){
  //Initialising publishing_nodes.
  pub_state_ = node->advertise<arc_msgs::State>("/state/"+pub_name_, 20);
  pub_path_ = node->advertise<nav_msgs::Path>("/"+pub_name_, 20);
}

void StateAndPathPublisher::publishWithQuaternion(Eigen::Vector3d position, Eigen::Vector4d quat, 
                           Eigen::Vector3d lin_vel, Eigen::Vector3d ang_vel, bool stop){
  //Updating class variables.
  array_position_ += 1;
  //Creating msgs.
  state_.pose.pose.position.x = position(0);
  state_.pose.pose.position.y = position(1);
  state_.pose.pose.position.z = position(2);
  state_.pose.pose.orientation.x = quat(0);
  state_.pose.pose.orientation.y = quat(1);
  state_.pose.pose.orientation.z = quat(2);
  state_.pose.pose.orientation.w = quat(3);
  state_.pose_diff.twist.linear.x = lin_vel(0);
  state_.pose_diff.twist.linear.y = lin_vel(1);
  state_.pose_diff.twist.linear.z = lin_vel(2);
  state_.pose_diff.twist.angular.x = ang_vel(0);
  state_.pose_diff.twist.angular.y = ang_vel(1);
  state_.pose_diff.twist.angular.z = ang_vel(2);
  state_.current_arrayposition = array_position_;
  state_.stop = stop;
  path_.poses.push_back(state_.pose);
  //Publishen.
  pub_state_.publish(state_);
  pub_path_.publish(path_);
  //Writing path File.
  stream_ <<array_position_<<" "<<
           position(0)<<" "<<position(1)<<" "<<position(2)<<" "<<
           quat(0)<<" "<<quat(1)<<" "<<quat(2)<<" "<<quat(3)<<" "<<
           lin_vel(0)<<" "<<lin_vel(1)<<" "<<lin_vel(2)<<" "<<
           ang_vel(0)<<" "<<ang_vel(1)<<" "<<ang_vel(2)<<" "<<stop<<"|";
}
}//namespace arc_state_estimation.