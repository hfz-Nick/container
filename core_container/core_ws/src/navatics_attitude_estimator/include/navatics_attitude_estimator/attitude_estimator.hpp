#ifndef NAVATICSATTITUDEESTIMATOR_ATTITUDEESTIMATOR_H
#define NAVATICSATTITUDEESTIMATOR_ATTITUDEESTIMATOR_H

#include <rclcpp/rclcpp.hpp>

#include "navatics_attitude_estimator/attitude_predictor.hpp"
#include "navatics_attitude_estimator/orientation.hpp"

#include <chrono>
#include <iostream>
#include <string>

namespace navatics_attitude_estimator{

class PredictorParametersStorage{

  /*
  This class is designed to contain all predictor parameters to be used for predictor initialization
  */

  public:
  PredictorParametersStorage(){};
  
  bool use_predictor = false;
  std::string name = "mpu00";
  uint8_t dev_addr = 0x68;
  std::string dev_port = "/dev/i2c0";
  std::string sensor_dir = "";
  
  bool use_gyro = false;
  float gyro_hz = 100.0;
  float gyro_gain = 0.9;
  float gyro_vel_gain = 0.95;
  int gyro_max = 250; // dps
  
  bool use_acc = false;
  float acc_hz = 100.0;
  float acc_gain = 0.1;
  int acc_max = 2; // G
  
  bool use_mag = false;
  float mag_hz = 50.0;
  float mag_gain = 0.1;

  void print_params(){
    std::cout << "name: " << this->name << "\n";
    std::cout << "dev_addr: " << (int)this->dev_addr << "\n";
    std::cout << "dev_port: " << this->dev_port << "\n";
    std::cout << "in use: " << this->use_predictor << "\n";
    std::cout << "sensor_dir: " << this->sensor_dir << "/" << this->name << "\n";
    
    if (this->use_predictor){
      
      if (this->use_gyro){
        std::cout << "gyro_hz: " << this->gyro_hz << "\n";
        std::cout << "gyro_vel_gain: " << this->gyro_vel_gain << "\n";
        std::cout << "gyro_gain: " << this->gyro_gain << "\n";
        std::cout << "gyro_max: " << this->gyro_max << "\n";
      }  

      if (this->use_acc){
        std::cout << "acc_hz: " << this->acc_hz << "\n";
        std::cout << "acc_gain: " << this->acc_gain << "\n";
        std::cout << "acc_max: " << this->acc_max << "\n";
      }  

      if (this->use_mag){
        std::cout << "mag_hz: " << this->mag_hz << "\n";
        std::cout << "mag_gain: " << this->mag_gain << "\n";
      }  

    } // if predictor is used
    
  }

  uint8_t get_acc_range(){
    uint8_t result = ACC_RANGE_2G;
    switch(this->acc_max){
      case(2): result=ACC_RANGE_2G; break;
      case(4): result=ACC_RANGE_4G; break;
      case(8): result=ACC_RANGE_8G; break;
      case(16): result=ACC_RANGE_16G; break;
    } // switch 
    return result;
  } // get_acc_range

  uint8_t get_gyro_range(){
    uint8_t result = GYRO_RANGE_250DPS;
    switch(this->gyro_max){
      case(250): result=GYRO_RANGE_250DPS; break;
      case(500): result=GYRO_RANGE_500DPS; break;
      case(1000): result=GYRO_RANGE_1000DPS; break;
      case(2000): result=GYRO_RANGE_2000DPS; break;
    } // switch 
    return result;
  } // get_gyro_range
  
}; // class ParameterStorage

class AttitudeEstimator{

  /*
  This class is designed to:
  1. Create loops that drives message publishing for sensors and state
  2. Initialize and link states to predictors
  3. Load important parameters for running attitude estimator
  4. Overall wrapper of the whole orientation estimation function
  */

  public:
  AttitudeEstimator(rclcpp::Node * node);

  private:
  float state_update_hz = 100.0;
  float sensor_update_hz = 50.0;
  
  Orientation *orientation;
  AttitudePredictor *predictor_00;
  AttitudePredictor *predictor_01;

  // params handler
  void declare_params(rclcpp::Node * node);
  void get_params(rclcpp::Node * node);
  void check_params();
  PredictorParametersStorage def_params;
  PredictorParametersStorage p0_params;
  PredictorParametersStorage p1_params;

  // publish timer
  rclcpp::TimerBase::SharedPtr state_pub_timer;
  void state_pub_callback();
  rclcpp::TimerBase::SharedPtr sensor_data_pub_timer;
  void sensor_data_pub_callback();

  // calibration
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mag_calib_sub;
  void mag_calib_sub_callback(const std_msgs::msg::Bool::SharedPtr msg);
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gyro_calib_sub;
  void gyro_calib_sub_callback(const std_msgs::msg::Bool::SharedPtr msg);
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr calib_status_pub;
  float calibration_check_frequency = 5.0f; // Hz
  
  // time handler
  std::chrono::time_point<std::chrono::system_clock> last_calibration_check_time;
  std::chrono::time_point<std::chrono::system_clock> get_system_time();
  float get_time_diff(auto end_time, auto start_time);


}; // class AttitudeEstimator

} // namespace navatics_attitude_estimator

#endif // NAVATICSATTITUDEESTIMATOR_ATTITUDEESTIMATOR_H
