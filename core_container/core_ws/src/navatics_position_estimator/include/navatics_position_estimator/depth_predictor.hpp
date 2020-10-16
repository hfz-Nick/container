#ifndef NAVATICSPOSITIONESTIMATOR_DEPTHPREDICTOR_H
#define NAVATICSPOSITIONESTIMATOR_DEPTHPREDICTOR_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include "navatics_position_estimator/position.hpp"

#include "navatics_i2c/devices/MS5837.h"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>

#include <Eigen/Eigen>

#define P_ATM 1.01325  // BAR
#define BAR_TO_METER 10.19977334

namespace navatics_position_estimator{

class DepthPredictor{
  
  /*
  This class is designed to:
  1. Create loop that drives data-reading from I2C devices and update sensor readings to
     the latest readings for position calculation, and update the position and velocity
  */
  public:
  DepthPredictor(rclcpp::Node * node, std::string sensor_name = "psu00", uint8_t dev_addr =0x76, 
                 std::string dev_port = "/dev/i2c-0", float pressure_update_hz=100.0f);

  // initialize position
  void initialize(Position *state);
  void link_state(Position *state);
  void set_update_weights(float pos_weight = 1.0, float vel_weight = 1.0);
  
  // publish sensor data
  void publish_sensor_data();
  
  // psu_test
  int psu_test();

  private:
  // pressure sensor interface
  MS5837 psu;
  std::string sensor_name;
  std::string dev_port;
  uint8_t dev_addr;

  // loop
  float pressure_update_hz = 100.0;
  rclcpp::TimerBase::SharedPtr depth_timer;
  void depth_read_callback();

  // publishers
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pressure_publisher;

  // storage values
  float pressure;
  std::vector <float> depth_container;
  int max_depth_container_size = 10;
  std::vector <float> velocity_container;
  int max_velocity_container_size = 5;
  float depth, prev_depth, velocity;

  // helper functions and variables
  float pressure_to_depth(float pressure);
  float mean_filter(float depth, std::vector<float> * container, int max_container_size);
  std::chrono::time_point<std::chrono::system_clock> get_system_time();
  float get_time_diff(auto end_time, auto start_time);
  std::chrono::time_point<std::chrono::system_clock> start_time;
  
  // state and update gains
  Position *state;
  float pos_weight = 1.0;
  float vel_weight = 1.0;

  // calibration data and calibration file
  std::string calib_filename;
  float pressure_bias = 0.0;
  bool is_calibrated = false;
  bool is_calibration_finished = false;
  std::vector <float> calibration_array;
  int n_calib_array_len = 100;
  float median_distance_threshold = 0.3;
  void calibrate_pressure_sensor();

  // state initializations
  bool is_state_linked = false;
  bool init_state_after_calibration = false;
  void initialize_linked_state();
}; // class DepthEstimator

} // namespace navatics_position_estimator

#endif // NAVATICSPOSITIONESTIMATOR_DEPTHPREDICTOR_H
