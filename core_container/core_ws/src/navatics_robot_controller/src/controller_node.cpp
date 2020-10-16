#include "navatics_robot_controller/controller.hpp"

class ControllerNode : public rclcpp::Node
{
  public:
  ControllerNode() : Node("controller_node")
  {
    using namespace navatics_robot_controller;
    this->controller = new Controller(this);
  }
  
  private:
  navatics_robot_controller::Controller *controller;

}; // class ControllerNode

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
