#ifndef NAVATICSPOSITIONESTIMATOR_POSITIONESTIMATOR_H
#define NAVATICSPOSITIONESTIMATOR_POSITIONESTIMATOR_H

#include <rclcpp/rclcpp.hpp>
#include "navatics_position_estimator/position.hpp"
#include "navatics_position_estimator/depth_predictor.hpp"

#include <iostream>

namespace navatics_position_estimator{

class PositionEstimator{
  /*
  This class is designed to:
  1. Create loop that drives message publishing for sensors and states
  2. Initialize and link states to predictors
  3. Load important parameters for running attitude estimator
  4. Overall wrapper of the whole position estimation function
  */
  public:
  PositionEstimator(rclcpp::Node * node);

  private:
  DepthPredictor *depth_predictor;
  Position *position;

  // publish timer
  rclcpp::TimerBase::SharedPtr state_pub_timer;
  void state_pub_callback();
  rclcpp::TimerBase::SharedPtr sensor_pub_timer;
  void sensor_pub_callback();

  // parameters handler
  void declare_params(rclcpp::Node * node);
  void get_params(rclcpp::Node * node);
  void print_params();
  // update parameters
  float sensor_update_hz = 50.0f;
  float state_update_hz = 100.0f;
  // predictor parameters
  std::string predictor_name = "psu00";
  uint8_t predictor_dev_addr = 0x76;
  std::string predictor_dev_port = "/dev/i2c-0";
  float predictor_hz = 100.0;
  float predictor_pos_gain = 0.95;
  float predictor_vel_gain = 0.95;

}; // class PositionEstimator

} // namespace navatics_position_estimator

#endif // NAVATICSPOSITIONESTIMATOR_POSITIONESTIMATOR_H
