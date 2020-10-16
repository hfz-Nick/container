#ifndef NAVATICSPOSITIONESTIMATOR_POSITION_H
#define NAVATICSPOSITIONESTIMATOR_POSITION_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <Eigen/Eigen>

namespace navatics_position_estimator{

class Position{
  /*
  This class is designed to:
  1. Store current ROV position
  2. Update ROV position based on the new sensor data available
  3. Contain publisher for state, and function that can be called to publish states
  */

  public:
  Position(rclcpp::Node * node);
  
  void initialize(Eigen::Vector3f ff_position, Eigen::Vector3f ff_velocity);
 
  void update_position(Eigen::Vector3f ff_position, float weight);
  void update_velocity(Eigen::Vector3f ff_velocity, float weight);
 
  bool initialized();
  Eigen::Vector3f get_ff_position();
  Eigen::Vector3f get_ff_velocity();

  void publish_state();

  private:
  Eigen::Vector3f ff_position;
  Eigen::Vector3f ff_velocity;

  // flags
  bool is_initialized = false;

  // publisher
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr position_publisher;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr velocity_publisher;

}; // class Position

} // namespace navatics_position_estimator

#endif // NAVATICSPOSITIONESTIMATOR_POSITION_H
