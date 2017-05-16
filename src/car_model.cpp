#include "arc_state_estimation/car_model.hpp"

namespace arc_state_estimation{

CarModel::CarModel(){}

CarModel::~CarModel(){}

void CarModel::createPublisher(ros::NodeHandle* node){
    std::string topic;
    node->getParam("/topic/CAR_MODEL_VELOCITY", topic);
    pub_velocity_ = node->advertise<geometry_msgs::TwistStamped>(topic, 10);
    // pub_rotated_vel_ = node->advertise<geometry_msgs::TwistStamped>("rotated", 10);
}

double CarModel::getVelocity(){
    double mean_velocity = (velocity_left_ + velocity_right_)/2.0;
    return mean_velocity;
}

Eigen::Vector3d CarModel::getVelocityVector(){return car_velocity_vector_;}

void CarModel::setDistanceWheelAxis(float distance_axis){L_ = distance_axis;}

void CarModel::setLengthWheelAxis(float length_axis){B_ = length_axis;}

void CarModel::setSteeringAngle(float steering_angle){steering_angle_ = steering_angle;}

void CarModel::setVelocityLeft(float velocity_left){velocity_left_ = velocity_left;}

void CarModel::setVelocityRight(float velocity_right){velocity_right_ = velocity_right;}

void CarModel::updateModel(Eigen::Vector4d orientation){
    double v_x;
    double v_y;
    //std::cout << "The steering angle is " << steering_angle_ << std::endl;
    //Geometric calculations: Equal angular velocities and current center of rotation on
    //horizontal line from rear axle.
    if(fabs(steering_angle_) <= 0.01) {
        v_x = (velocity_right_ + velocity_left_ ) / 2;
        v_y = 0; 
    }
    
    else { 
        double v_rear = (velocity_right_ + velocity_left_ ) / 2;
        double beta = M_PI / 2 - fabs(steering_angle_);
        double r1 = L_ * tan(beta);
        double r2 = L_ / cos(beta);
        double w = v_rear / r1;
        double v_front = w * r2;
        v_x = cos(steering_angle_) * v_front;
        v_y = sin (steering_angle_) * v_front;
    }
    // //Find geometrics.
    // float a = fabs(L_/tan(steering_angle_));
    // float a_left = a - B_/2;
    // float a_right = a + B_/2;
    // float R = fabs(L_/sin(steering_angle_));
    // float R_left = sqrt(a_left*a_left + L_*L_);
    // float R_right = sqrt(a_right*a_right + L_*L_);
    // //Rear axis.
    // double w_left = velocity_left_/a_left;
    // double w_right = velocity_right_/a_right;
    // double w_rear = (w_left+w_right)/2;
    // //Front axis.
    // // float v_center_left = velocity_left_ * R / R_left; 
    // // float v_center_right = velocity_right_ * R / R_right; 
    // // float v_center = (v_center_right+v_center_left)/2;
    // // float w_front = fabs(sin(steering_angle_) / B_);
    // float w_front = w_rear;
    // //Get velocities.
    // v_x = (w_rear+w_front)/2*a;
    // v_y = (w_rear+w_front)/2*L_;
    // if(steering_angle_<0) v_y = -v_y;
    //Updating velocity vector.
    car_velocity_vector_(0) = v_y;
    car_velocity_vector_(1) = 0.1908*v_x;
    car_velocity_vector_(2) = 0.981627*v_x;
    //Publishing.
    geometry_msgs::TwistStamped twist;
    //twist.twist.linear.x = car_velocity_vector_(0);
    //twist.twist.linear.y  = car_velocity_vector_(1);
    //twist.twist.linear.z  = car_velocity_vector_(2);
    twist.header.stamp = time_stamp_;
    twist.twist.linear.x = car_velocity_vector_(0);
    twist.twist.linear.y  = car_velocity_vector_(1);
    twist.twist.linear.z  = car_velocity_vector_(2);
    pub_velocity_.publish(twist);
    usleep(100000);    

    // Eigen::Vector4d init_quat(0.78,-0.037,0.004,-0.624);
    // Eigen::Vector3d init_euler = arc_tools::transformEulerQuaternionVector(init_quat);
    // Eigen::Matrix3d init_rot = arc_tools::getRotationMatrix(init_euler);
    // Eigen::Vector3d vel(v_x,v_y,0);
    // Eigen::Vector3d rotated_vel = init_rot*vel;
    // Eigen::Vector3d rotated_vel(v_y,0,v_x);
    // geometry_msgs::TwistStamped rot_twist;
    // rot_twist.twist.linear.x = rotated_vel(0);
    // rot_twist.twist.linear.y  = rotated_vel(1);
    // rot_twist.twist.linear.z  = rotated_vel(2);
    // pub_rotated_vel_.publish(rot_twist);


}

void CarModel::setTime(ros::Time time_stamp){
    time_stamp_ = time_stamp;
}

}//namespace arc_state_estimation.