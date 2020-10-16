#include "navatics_position_estimator/position.hpp"

namespace navatics_position_estimator{

Position::Position(rclcpp::Node * node){
  this->ff_position << 0,0,0;
  this->ff_velocity << 0,0,0;

  // publishers
  this->position_publisher = node->create_publisher<geometry_msgs::msg::Vector3>("state/position", 100);
  this->velocity_publisher = node->create_publisher<geometry_msgs::msg::Vector3>("state/velocity", 100);
} // constructor

void Position::initialize(Eigen::Vector3f ff_position, Eigen::Vector3f ff_velocity){
  this->ff_position = ff_position;
  this->ff_velocity = ff_velocity;
  this->is_initialized = true;
} // initialize

void Position::update_position(Eigen::Vector3f ff_position, float weight){
  if (this->initialized())
    this->ff_position = this->ff_position*(1-weight) + ff_position*weight;
} // update_position

void Position::update_velocity(Eigen::Vector3f ff_velocity, float weight){
  if (this->initialized())
    this->ff_velocity = this->ff_velocity*(1-weight) + ff_velocity*weight;
} // update_velocity

Eigen::Vector3f Position::get_ff_position(){
  return this->ff_position;
} // get_ff_position

Eigen::Vector3f Position::get_ff_velocity(){
  return this->ff_velocity;
} // get_ff_velocity

bool Position::initialized(){
  return this->is_initialized;
} //initialized()

void Position::publish_state(){
  auto message = geometry_msgs::msg::Vector3();
  message.x = this->ff_position[0];
  message.y = this->ff_position[1];
  message.z = this->ff_position[2];
  this->position_publisher->publish(message);
  message.x = this->ff_velocity[0];
  message.y = this->ff_velocity[1];
  message.z = this->ff_velocity[2];
  this->velocity_publisher->publish(message);
}

} // namespace navatics_position_estimator
