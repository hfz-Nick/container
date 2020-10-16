#include "navatics_attitude_estimator/attitude_estimator.hpp"

class AttitudeEstimatorNode : public rclcpp::Node
{
  public:
  AttitudeEstimatorNode()
  : Node("attitude_estimator_node")
  {
    using namespace navatics_attitude_estimator;
    this->estimator = new AttitudeEstimator(this);
  } // constructor
  
  private:
  navatics_attitude_estimator::AttitudeEstimator *estimator;

  
}; // class AttitudeEstimatorNode

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AttitudeEstimatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


