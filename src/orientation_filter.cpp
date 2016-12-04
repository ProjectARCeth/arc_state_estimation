#include "arc_state_estimation/orientation_filter.hpp"

using namespace arc_state_estimation;

OrientationFilter::OrientationFilter(){};

void OrientationFilter::initOrientationFilter(const Eigen::VectorXd& x0, double weight){
  //Initialising state.
  x_ = x0;
  angle_first_estimation_ = Eigen::MatrixXd::Zero(3,1);
  angle_second_estimation_ = Eigen::MatrixXd::Zero(3,1);
  //Initialising Transformation matrix.
  H_ = Eigen::MatrixXd::Zero(6,6);
  //Init time.
  time_.start();
  initialized_ = true;
  weight_ = weight;
}

void OrientationFilter::updateMatrices(){
  //Rotation from body to global frame and Transformation matrix from 
  //rotational velocity aroud axes to euler velocities
  //(https://www.princeton.edu/~stengel/MAE331Lecture9.pdf).
  Eigen::Matrix<double,3,3> Trafomatrix = getAngularVelocityTransformationMatrix(x_.segment<3>(0));
  //Calculating Jacobian matrix.
  Eigen::Matrix<double,3,3>Jacobian;
  Jacobian(0,0)=0.0;Jacobian(0,1)=-cos(x_(1))*x_(4);Jacobian(0,2)=0.0;
  Jacobian(1,0)=cos(x_(0))*cos(x_(1))*x_(3);Jacobian(1,1)=-sin(x_(0))*sin(x_(1))*x_(4);Jacobian(1,2)=0.0;
  Jacobian(2,0)=sin(x_(0))*cos(x_(1))*x_(3);Jacobian(2,1)=cos(x_(0))*sin(x_(1))*x_(4);Jacobian(2,2)=0.0;
  //Updating measurement matrix.
  H_.block<3,3>(0,0) = Jacobian;
  H_.block<3,3>(3,3) = Trafomatrix;
}

void OrientationFilter::update(const sensor_msgs::Imu::ConstPtr& imu_data){
  //Time Interval.
  timestep_ = time_.getTimestep();
  //Update Matrices.
  updateMatrices();
  //Getting imu-measurements.
  current_measurements_(0) = imu_data->linear_acceleration.x;
  current_measurements_(1) = imu_data->linear_acceleration.y;
  current_measurements_(2) = imu_data->linear_acceleration.z;
  current_measurements_(3) = imu_data->angular_velocity.x;
  current_measurements_(4) = imu_data->angular_velocity.y;
  current_measurements_(5) = imu_data->angular_velocity.z;
  //Normalising acceleration measurements.
  double ax = current_measurements_(0);
  double ay = current_measurements_(1);
  double az = current_measurements_(2);
  double norm = sqrt(ax*ax + ay*ay + az*az);
  current_measurements_.segment<3>(0) = current_measurements_.segment<3>(0)/norm;
  //Filtering measurements.
  current_measurements_= firstOrderLowPassIIRFilter(current_measurements_,
                                                    last_measurements_, exp(-timestep_/0.5));
  last_measurements_ = current_measurements_;
  // First Estimation (angle = angular_velocity_measurement * dt).
  Eigen::Vector3d omega = H_.block<3,3>(3,3)*current_measurements_.segment<3>(3);
  angle_first_estimation_ += timestep_*omega;
  x_(0) += timestep_*omega(0);
  x_(1) += timestep_*omega(1);
  x_(2) += timestep_*omega(2);

  // //Second estimation (gravitation orientation).
  // Eigen::Vector3d gravity_orientation = H_.block<3,3>(0,0).inverse()*current_measurements_.segment<3>(0);
  // double elementZero = x_(0)*x_(1)*x_(2)*x_(3)*x_(4)*x_(5);
  // if(elementZero <= 0.05){}
  // else{angle_second_estimation_ += gravity_orientation;}
  // //Weighting.
  // x_.segment<3>(0) = weight_*angle_first_estimation_ + (1-weight_)*angle_second_estimation_;
  // //Updating angular velocities.
  // x_.segment<3>(3) = omega;
}

Eigen::VectorXd OrientationFilter::getState(){
    return x_;
}

