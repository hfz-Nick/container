#include <rclcpp/rclcpp.hpp>

#include "navatics_server/mavlink_server.hpp"

class MAVLinkServerTestNode: public rclcpp::Node
{
  public:
  MAVLinkServerTestNode():
  Node("mavlink_server_test_node")
  {
    using namespace navatics_server;
    this->srv = new MAVLinkServer(this, "192.168.100.100", 15001, 15000);
  } // constructor

  private:
  navatics_server::MAVLinkServer *srv;
}; // class MAVLinkServerTestNode

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MAVLinkServerTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
