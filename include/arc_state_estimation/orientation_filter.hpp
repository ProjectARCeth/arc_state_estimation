#ifndef ORIENTATION_FILTER_ARC_STATE_ESTIMATION_HPP
#define ORIENTATION_FILTER_ARC_STATE_ESTIMATION_HPP

#include "arc_tools/coordinate_transform.hpp"
#include "arc_tools/signal_filter.hpp"
#include "arc_tools/timing.hpp"

#include "ros/ros.h"
#include "Eigen/Dense"
#include <time.h>
#include <cmath>

#include "sensor_msgs/Imu.h"

namespace arc_state_estimation{

class OrientationFilter{
 public:
  OrientationFilter();
  void initOrientationFilter(const Eigen::VectorXd& x0, double weight);
  void update(const sensor_msgs::Imu::ConstPtr& imu_data);
  Eigen::VectorXd getState();

private:
  void updateMatrices();
  Eigen::Matrix<double,6,1> current_measurements_;
  Eigen::Matrix<double,6,1> last_measurements_;
  Eigen::Vector3d angle_first_estimation_;
  Eigen::Vector3d angle_second_estimation_;
  Eigen::VectorXd x_;
  Eigen::MatrixXd H_;
  bool initialized_;
  double timestep_;
  arc_tools::Clock time_;
  double weight_;
};
}//namespace arc_state_estimation.

#endif