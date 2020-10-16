#ifndef NAVATICSATTITUDEESTIMATOR_ORIENTATION_H
#define NAVATICSATTITUDEESTIMATOR_ORIENTATION_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <Eigen/Eigen>
#include <iostream>
#include <random>

namespace navatics_attitude_estimator{

class Orientation{
  /*
  This class is designed to:
  1. Store the current ROV orientation
  2. Update the ROV orientation based on the new sensor data available
  3. Contain publisher for state, and a function that can be called to publish the state
  */

  public:
  Orientation(rclcpp::Node * node);

  // update unit test
  int update_equation_unit_test(float weight = 0.9, float q_threshold = 0.1, float w_threshold = 0.1);
  
  // initialize orientation
  void initialize(Eigen::Quaternionf q, Eigen::Vector3f w);

  // update state
  void update_orientation(Eigen::Quaternionf q, float weight);
  void update_angular_velocity(Eigen::Vector3f w, float weight);

  // member access functions
  Eigen::Quaternionf get_orientation();
  Eigen::Vector3f get_angular_velocity();
  bool initialized();

  // debugging tools
  void print_quaternion(Eigen::Quaternionf q);
  
  // publish state
  void publish_state();
  
  private:
  // Orientation Object
  Eigen::Quaternionf orientation;
  Eigen::Vector3f angular_velocity;

  // publishers
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr angular_velocity_publisher;
  rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr orientation_publisher;
  
  // flags
  bool is_initialized = false;

}; //class Orientation

} // namespace navatics_attitude_estimator

#endif // NAVATICSATTITUDEESTIMATOR_ORIENTATION_H
