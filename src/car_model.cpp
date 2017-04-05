#include "arc_state_estimation/car_model.hpp"

namespace arc_state_estimation{

CarModel::CarModel(){}

CarModel::~CarModel(){}

void CarModel::createPublisher(ros::NodeHandle* node){
    std::string topic;
    node->getParam("/topic/CAR_MODEL_VELOCITY", topic);
    pub_velocity_ = node->advertise<geometry_msgs::TwistWithCovarianceStamped>(topic, 10);
}

double CarModel::getVelocity(){
    double mean_velocity = (velocity_left_ + velocity_right_)/2.0;
    return mean_velocity;
}

void CarModel::setDistanceWheelAxis(float distance_axis){L_ = distance_axis;}

void CarModel::setLengthWheelAxis(float length_axis){B_ = length_axis;}

void CarModel::setSteeringAngle(float steering_angle){steering_angle_ = steering_angle;}

void CarModel::setVelocityLeft(float velocity_left){velocity_left_ = velocity_left;}

void CarModel::setVelocityRight(float velocity_right){velocity_right_ = velocity_right;}

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
    //Convert to global frame.
    Eigen::Vector3d orientation_euler = arc_tools::transformEulerQuaternionVector(orientation);
    Eigen::Matrix3d rotation_matrix = arc_tools::getRotationMatrix(orientation_euler);
    Eigen::Vector3d v_global = rotation_matrix * velocity_local; 
    //Publishing.
    geometry_msgs::TwistWithCovarianceStamped twist;
    twist.twist.twist.linear.x = v_global(0);
    twist.twist.twist.linear.y  = v_global(1);
    twist.twist.twist.linear.z  = v_global(2);
    pub_velocity_.publish(twist);
}

}//namespace arc_state_estimation.