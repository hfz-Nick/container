#include "navatics_server/endpoints.hpp"

class EndpointsNode : public rclcpp::Node
{
  public:
  EndpointsNode() : Node("endpoints_node"){
    using namespace navatics_server;
    this->endpoints = new Endpoints(this);
  } // constructor

  private:
  navatics_server::Endpoints *endpoints;
}; // class EndpointsNode

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EndpointsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} // main
