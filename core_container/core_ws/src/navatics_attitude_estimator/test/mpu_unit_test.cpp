#include "navatics_attitude_estimator/attitude_predictor.hpp"

class MPUUnitTestNode : public rclcpp::Node
{
  public:
  MPUUnitTestNode()
  : Node("mpu_unit_test")
  {
    
    using namespace navatics_attitude_estimator;
    // MPU0, ID: 0x69, Port: i2c-0
    this->predictor_0 = new AttitudePredictor(this, 0x69, "/dev/i2c-0", "mpu00", "/home/ros/sensors",
                                              true, ACC_RANGE_16G, 1.0, 
                                              true, GYRO_RANGE_250DPS, 1.0,
                                              true, 1.0, false);
    // MPU1, ID: 0x68, Port: i2c-2
    this->predictor_1 = new AttitudePredictor(this, 0x68, "/dev/i2c-2", "mpu01", "/home/ros/sensors",
                                             true, ACC_RANGE_16G, 1.0, 
                                             true, GYRO_RANGE_2000DPS, 1.0,
                                             true, 1.0, false);
  }
  
  int mpu_test(){
    int ret_val = 0;
    ret_val += this->predictor_0->mpu_test();
    ret_val += this->predictor_1->mpu_test()*2;
    return ret_val;
  }

  private:
  navatics_attitude_estimator::AttitudePredictor *predictor_0;
  navatics_attitude_estimator::AttitudePredictor *predictor_1;
  
}; // class MPUUnitTestNode

int main (int argc, char * argv[]){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MPUUnitTestNode>();
  int ret_val = node->mpu_test();
  rclcpp::shutdown();
  std::cout << "Test exited with code " << ret_val << "\n";
  return ret_val;
} // main
