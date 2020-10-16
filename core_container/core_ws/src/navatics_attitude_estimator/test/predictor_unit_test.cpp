#include "navatics_attitude_estimator/attitude_predictor.hpp"

class PredictorUnitTestNode : public rclcpp::Node
{
  public:
  PredictorUnitTestNode()
  : Node("predictor_unit_test")
  {
    
    using namespace navatics_attitude_estimator;
    // MPU0, ID: 0x68
    this->predictor = new AttitudePredictor(this, 0x68, "/dev/i2c-0", "mpu00", "/home/ros/sensors",
                                            true, ACC_RANGE_16G, 1.0, 
                                            true, GYRO_RANGE_250DPS, 1.0,
                                            true, 1.0, false, true);
    this->orientation = new Orientation(this);
  }
  
  int test_predictor(){
    return this->predictor->get_rotation_test();
  }
  
  int test_orientation(){
    return this->orientation->update_equation_unit_test();
  }
  
  private:
  navatics_attitude_estimator::AttitudePredictor *predictor;
  navatics_attitude_estimator::Orientation *orientation;
  
}; // class MPUUnitTestNode

int main (int argc, char * argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PredictorUnitTestNode>();
  uint8_t p_result = node->test_predictor();
  uint8_t o_result = node->test_orientation();
  int result = (p_result << 8 | o_result);
  std::cout << "Test completed with exit code: " << result << "\n";
  rclcpp::shutdown();
  return result;
} // main
