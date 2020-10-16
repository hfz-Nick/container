#include "navatics_server/endpoints.hpp"

namespace navatics_server{

Endpoints::Endpoints(rclcpp::Node * node){
  this->declare_params(node);
  this->get_params(node);
  this->server = new MAVLinkServer(node, this->remote_ip, this->remote_port, this->local_port);

  // set loop rate
  this->server->set_loop_rate(MAVLinkServer::STATE_UPDATE_HZ, this->pub_rate_state);
  this->server->set_loop_rate(MAVLinkServer::REFSTATE_UPDATE_HZ, this->pub_rate_refstate);
} // constructor

void Endpoints::declare_params(rclcpp::Node * node){
  node->declare_parameter("onshore_ip");
  node->declare_parameter("onshore_port");
  node->declare_parameter("uusv_port");
  node->declare_parameter("mavlink_pub_rate.state");
  node->declare_parameter("mavlink_pub_rate.refstate");
} // declare_params

void Endpoints::get_params(rclcpp::Node * node){
  std::string broadcast_ip = "255.255.255.255";
  node->get_parameter_or("onshore_ip", this->remote_ip, broadcast_ip);
  node->get_parameter_or("onshore_port", this->remote_port, 15000);
  node->get_parameter_or("uusv_port", this->local_port, 15001);
  node->get_parameter_or("mavlink_pub_rate.state", this->pub_rate_state, 20.0f);
  node->get_parameter_or("mavlink_pub_rate.refstate", this->pub_rate_refstate, 20.0f);
  std::cout << "[PARAMS] remote_ip: " << this->remote_ip << "\n";
  std::cout << "[PARAMS] remote_port: " << this->remote_port << "\n";
  std::cout << "[PARAMS] local_port: " << this->local_port << "\n";
  std::cout << "[PARAMS] Update Rates:\n";
  std::cout << "-- State Update Rate: " << this->pub_rate_state << "\n";
  std::cout << "-- RefState Update Rate: " << this->pub_rate_refstate << "\n";
} // get_params

} // namespace navatics_server
