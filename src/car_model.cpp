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
    if(fabs(steering_angle_) <= 0.01 && steering_angle_ >= 0) steering_angle_ = 0.01;
    if(fabs(steering_angle_) <= 0.01 && steering_angle_ < 0) steering_angle_ = -0.01;
    //Find geometrics.
    float a = fabs(L_/tan(steering_angle_));
    float a_left = a - B_/2;
    float a_right = a + B_/2;
    float R = fabs(L_/sin(steering_angle_));
    float R_left = sqrt(a_left*a_left + L_*L_);
    float R_right = sqrt(a_right*a_right + L_*L_);
    //Rear axis.
    double w_left = velocity_left_/a_left;
    double w_right = velocity_right_/a_right;
    double w_rear = (w_left+w_right)/2;
    //Front axis.
    // float v_center_left = velocity_left_ * R / R_left; 
    // float v_center_right = velocity_right_ * R / R_right; 
    // float v_center = (v_center_right+v_center_left)/2;
    // float w_front = fabs(sin(steering_angle_) / B_);
    float w_front = w_rear;
    //Get velocities.
    double v_x = (w_rear+w_front)/2*a;
    double v_y = (w_rear+w_front)/2*L_;
    if(steering_angle_<0) v_y = -v_y;
    //Publishing.
    geometry_msgs::TwistWithCovarianceStamped twist;
    twist.twist.twist.linear.x = v_x;
    twist.twist.twist.linear.y  = v_y;
    twist.twist.twist.linear.z  = 0;
    pub_velocity_.publish(twist);
}

}//namespace arc_state_estimation.