#include "navatics_position_estimator/position_estimator.hpp"

class PositionEstimatorNode : public rclcpp::Node
{
  public:
  PositionEstimatorNode()
  : Node("position_estimator_node")
  {
    using namespace navatics_position_estimator;
    this->estimator = new PositionEstimator(this);
  } // constructor

  private:
  navatics_position_estimator::PositionEstimator *estimator;


}; // class AttitudeEstimatorNode

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PositionEstimatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

