#ifndef NAVATICSSERVER_ENDPOINTS_H
#define NAVATICSSERVER_ENDPOINTS_H

#include <rclcpp/rclcpp.hpp>

#include "navatics_server/mavlink_server.hpp"

namespace navatics_server{

class Endpoints{

  public:
  Endpoints(rclcpp::Node * node);

  private:
  MAVLinkServer *server;
  
  void get_params(rclcpp::Node * node);
  void declare_params(rclcpp::Node * node);
  
  std::string remote_ip;
  int remote_port;
  int local_port;

  // update rate of routine messages from server to ground control station
  float pub_rate_state;
  float pub_rate_refstate;

}; // class Endpoints

} // namespace navatics_server

#endif // NAVATICSSERVER_ENDPOINTS_H
