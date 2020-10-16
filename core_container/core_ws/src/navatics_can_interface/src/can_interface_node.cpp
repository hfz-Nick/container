#include "navatics_can_interface/can_interface_manager.hpp"

class CANInterfaceNode : public rclcpp::Node
{
  public:
  CANInterfaceNode() : Node("can_interface_node")
  {
    using namespace navatics_can_interface;
    this->manager = new CANInterfaceManager(this);
  }

  private:
  navatics_can_interface::CANInterfaceManager *manager;

}; // class ControllerNode

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CANInterfaceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

