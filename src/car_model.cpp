#include "arc_state_estimation/car_model.hpp"

namespace arc_state_estimation{

CarModel::CarModel(float distance_wheels, float length_axis){
  //Setting vehicle constants.
  L_ = distance_wheels;
  B_ = length_axis;
}

void CarModel::createPublisher(ros::NodeHandle* node){
    pub_velocity_ = node->advertise<geometry_msgs::Transform>("car_model_velocity", 10);
}

void CarModel::updateModel(Eigen::Vector4d orientation){
    //Geometric calculations: Equal angular velocities and current center of rotation on
    //horizontal line from rear axle.
    float R = L_ / sin(steering_angle_);
    float a = L_ / tan(steering_angle_);
    float a_l = a - B_/2;
    float a_r = a + B_/2;
    float R_l = sqrt(a_l*a_l + L_*L_);
    float R_r = sqrt(a_r*a_r + L_*L_);
    float v_front_mean = velocity_left_ * R / R_l; 
    float v_back = v_front_mean * cos(steering_angle_);
    float omega = sin(steering_angle_) / B_;
    //Velocities in local frame.
    Eigen::Vector3d velocity_local(v_back, 0, 0);
    Eigen::Vector3d omega_local(0, 0, omega);
    //Convert to global frame.
    Eigen::Vector3d orientation_euler = arc_tools::transformEulerQuaternionVector(orientation);
    Eigen::Matrix3d rotation_matrix = arc_tools::getRotationMatrix(orientation_euler);
    Eigen::Vector3d v_global = rotation_matrix * velocity_local; 
    Eigen::Vector3d omega_global = rotation_matrix * omega_local;
    //Publishing.
    geometry_msgs::Transform transformation;
    transformation.translation.x = v_global(0);
    transformation.translation.y = v_global(1);
    transformation.translation.z = v_global(2);
    transformation.rotation = arc_tools::transformQuaternionEulerMsg(omega_global);
    pub_velocity_.publish(transformation);
}

void CarModel::set_steering_angle(float steering_angle){steering_angle_ = steering_angle;}

void CarModel::set_velocity_left(float velocity_left){velocity_left_ = velocity_left;}

void CarModel::set_velocity_right(float velocity_right){velocity_right_ = velocity_right;}

}//namespace arc_state_estimation.