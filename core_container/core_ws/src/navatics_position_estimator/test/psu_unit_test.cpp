#include "navatics_position_estimator/depth_predictor.hpp"

class PSUUnitTestNode : public rclcpp::Node
{
  public:
  PSUUnitTestNode()
  : Node("psu_unit_test")
  {
    using namespace navatics_position_estimator;
    this->predictor = new DepthPredictor(this);   
  }

  int psu_unit_test(){
    this->predictor->psu_test();
  }

  private:
  navatics_position_estimator::DepthPredictor *predictor;

}; // class PSUUnitTestNode

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PSUUnitTestNode>();
  int ret_val = node->psu_unit_test();
  rclcpp::shutdown();
  std::cout << "Test exited with exit code " << ret_val << "\n";
  return ret_val;
} // main
