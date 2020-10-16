#include "navatics_position_estimator/position_estimator.hpp"

namespace navatics_position_estimator{

PositionEstimator::PositionEstimator(rclcpp::Node * node){
  this->declare_params(node);
  this->get_params(node);
  this->print_params();

  // initialize predictor and state
  this->depth_predictor = new DepthPredictor(node, this->predictor_name, this->predictor_dev_addr,
                                             this->predictor_dev_port, this->predictor_hz);
  this->position = new Position(node);
  this->depth_predictor->initialize(this->position);
  this->depth_predictor->set_update_weights(this->predictor_pos_gain, this->predictor_vel_gain);
  
  // initialize timers
  std::chrono::milliseconds state_duration_ms((int)(1000/this->state_update_hz));
  this->state_pub_timer = node->create_wall_timer(
                            state_duration_ms,
                            std::bind(&PositionEstimator::state_pub_callback, this)
                          );

  std::chrono::milliseconds sensor_duration_ms((int)(1000/this->sensor_update_hz));
  this->sensor_pub_timer = node->create_wall_timer(
                            sensor_duration_ms,
                            std::bind(&PositionEstimator::sensor_pub_callback, this)
                          );

} // constructor

void PositionEstimator::state_pub_callback(){
  if (this->position->initialized()){
    this->position->publish_state();
  }
}

void PositionEstimator::sensor_pub_callback(){
  this->depth_predictor->publish_sensor_data();
}

void PositionEstimator::declare_params(rclcpp::Node * node){
  // general_params
  node->declare_parameter("sensor_update_hz");
  node->declare_parameter("state_update_hz");
  // predictor params
  node->declare_parameter("predictor.name");
  node->declare_parameter("predictor.dev_addr");
  node->declare_parameter("predictor.dev_port");
  // sensor parameter
  node->declare_parameter("predictor.pressure.hz");
  node->declare_parameter("predictor.pressure.gain");
} // declare_params

void PositionEstimator::get_params(rclcpp::Node * node){
  // get optional general parameters (update hz)
  node->get_parameter_or("sensor_update_hz", this->sensor_update_hz, 50.0f);
  node->get_parameter_or("state_update_hz", this->state_update_hz, 100.0f);
  //get predictor params
  node->get_parameter("predictor.name", this->predictor_name);
  node->get_parameter("predictor.dev_addr", this->predictor_dev_addr);
  node->get_parameter("predictor.dev_port", this->predictor_dev_port);
  node->get_parameter_or("predictor.pressure.hz", this->predictor_hz, 100.0f);
  node->get_parameter_or("predictor.pressure.pos_gain", this->predictor_pos_gain, 1.0f);
  node->get_parameter_or("predictor.pressure.vel_gain", this->predictor_vel_gain, 1.0f);
} // get_params

void PositionEstimator::print_params(){
  std::cout << "sensor_update_hz: " << this->sensor_update_hz << "\n";
  std::cout << "state_update_hz: " << this->state_update_hz << "\n";
  std::cout << "predictor_name: " << this->predictor_name << "\n";
  std::cout << "predictor_dev_addr: " << (int)this->predictor_dev_addr << "\n";
  std::cout << "predictor_dev_port: " << this->predictor_dev_port << "\n";
  std::cout << "predictor_hz: " << this->predictor_hz << "\n";
  std::cout << "predictor_pos_gain: " << this->predictor_pos_gain << "\n";
  std::cout << "predictor_vel_gain: " << this->predictor_vel_gain << "\n";
}

} // namespace navatics_position_estimator
