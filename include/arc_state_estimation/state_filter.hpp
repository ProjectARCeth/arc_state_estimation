#ifndef STATE_FILTER_ARC_STATE_ESTIMATION_HPP
#define STATE_FILTER_ARC_STATE_ESTIMATION_HPP

#include "ros/ros.h"
#include "Eigen/Dense"
#include <cmath>

#include "arc_msgs/State.h"

namespace arc_state_estimation{

class StateFilter{
public:
  //Constructor: Design Parameter for position filter (alpha_i = 1/(a*d_i^n)).
  StateFilter(double a, double n);
  //Position.
  void updateOrbslam(Eigen::Vector3d pos);
  //Position change.
  void updateGPS(Eigen::Vector3d pos);
  //Linear velocity, angular velocity, orientation.
  void updateRovio(Eigen::Vector<double,10> measurements);
  void updateRovioCovariance(Eigen::MatrixXd cov);
  //Velocity.
  void updateWheelSensor(double vel);
  //Filter.
  arc_msgs::State filtering();
private:
  //Measurements.
  Eigen::Vector3d orbslam_;
  Eigen::Vector3d gps_;
  Eigen::Vector<double,10> rovio_;
  double wheel_vel_;
  //State.
  Eigen::Vector3d pos_;
  Eigen::Vector4d orient_;
  Eigen::Vector3d lin_vel_;
  Eigen::Vector3d ang_vel_;
  //Weights & design parameters.
  double a_, n_, alpha_;
  double beta_;
};
}//namespace arc_state_estimation.

#endif