#ifndef STATE_FILTER_ARC_STATE_ESTIMATION_HPP
#define STATE_FILTER_ARC_STATE_ESTIMATION_HPP

#include "ros/ros.h"
#include "Eigen/Dense"
#include <cmath>

#include "sensor_msgs/Imu.h"

namespace arc_state_estimation{

class StateEstimation{
public:
  //Constructor.
  StateEstimation();
  //Position.
  void updateOrbslam(Eigen::Vector3d pos);
  void updateSegmatch(Eigen::Vector3d pos);
  void updateGPS(Eigen::Vector3d pos);
  //Linear velocity, angular velocity, orientation.
  void updateRovio(Eigen::Vector<double,10> measurements);
  //Velocity.
  void updateWheelSensor(double vel);

private:

};
}//namespace arc_state_estimation.

#endif