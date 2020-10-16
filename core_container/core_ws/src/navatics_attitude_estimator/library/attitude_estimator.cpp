#include "navatics_attitude_estimator/attitude_estimator.hpp"

namespace navatics_attitude_estimator{

AttitudeEstimator::AttitudeEstimator(rclcpp::Node * node)
{
  this->declare_params(node);
  this->get_params(node);
  this->check_params();
  this->predictor_00 = new AttitudePredictor(node, p0_params.dev_addr, p0_params.dev_port, 
                                             p0_params.name, p0_params.sensor_dir,
                                             p0_params.use_acc, p0_params.get_acc_range(), 
                                             p0_params.acc_hz, p0_params.use_gyro, 
                                             p0_params.get_gyro_range(), p0_params.gyro_hz,
                                             p0_params.use_mag, p0_params.mag_hz, false);
  this->predictor_00->set_update_weights(this->p0_params.gyro_gain, this->p0_params.gyro_vel_gain,
                                         this->p0_params.acc_gain, this->p0_params.mag_gain);
  this->orientation = new Orientation(node);
  this->predictor_00->initialize(this->orientation);
  if (p1_params.use_predictor){
    this->predictor_01 = new AttitudePredictor(node, p1_params.dev_addr, p1_params.dev_port, 
                                               p1_params.name, p1_params.sensor_dir,
                                               p1_params.use_acc, p1_params.get_acc_range(), 
                                               p1_params.acc_hz, p1_params.use_gyro, 
                                               p1_params.get_gyro_range(), p1_params.gyro_hz,
                                               p1_params.use_mag, p1_params.mag_hz, false);
    this->predictor_01->set_update_weights(this->p1_params.gyro_gain, this->p1_params.gyro_vel_gain,
                                           this->p1_params.acc_gain, this->p1_params.mag_gain);
    this->predictor_01->link_state(this->orientation);
  }
  
  // initialize timer
  std::chrono::milliseconds state_duration_ms((int)(1000/this->state_update_hz));
  this->state_pub_timer = node->create_wall_timer(
                            state_duration_ms,
                            std::bind(&AttitudeEstimator::state_pub_callback, this)
                          );
  std::chrono::milliseconds sensor_duration_ms((int)(1000/this->sensor_update_hz));
  this->sensor_data_pub_timer = node->create_wall_timer(
                                  sensor_duration_ms,
                                  std::bind(&AttitudeEstimator::sensor_data_pub_callback, this)
                                );
 
  this->calib_status_pub = node->create_publisher<std_msgs::msg::Bool>("mpu/calibrated", 10);
  this->last_calibration_check_time = this->get_system_time();
 
  using namespace std::placeholders;
  // initialize calibration message subscriber
  this->mag_calib_sub = node->create_subscription<std_msgs::msg::Bool> ("mpu/mag/calibrate", 10,
                           std::bind(&AttitudeEstimator::mag_calib_sub_callback, this, _1));
  this->gyro_calib_sub = node->create_subscription<std_msgs::msg::Bool> ("mpu/gyro/calibrate", 10,
                           std::bind(&AttitudeEstimator::gyro_calib_sub_callback, this, _1));
} // constructor

void AttitudeEstimator::state_pub_callback(){
  this->orientation->publish_state();
} // state_pub_callback

void AttitudeEstimator::sensor_data_pub_callback(){
  this->predictor_00->publish_sensor_data();
  if (this->p1_params.use_predictor)
    this->predictor_01->publish_sensor_data();
  
  // check calibration
  float dt = this->get_time_diff(this->get_system_time(), this->last_calibration_check_time);
  if (dt > 1.0f/this->calibration_check_frequency){
    bool is_calibrated = this->predictor_00->is_mpu_calibrated() and 
                         (this->predictor_01->is_mpu_calibrated() or 
                          not this->p1_params.use_predictor);
    auto msg = std_msgs::msg::Bool();
    msg.data = is_calibrated;
    this->calib_status_pub->publish(msg);
  }
} // sensor_data_pub_callback

void AttitudeEstimator::mag_calib_sub_callback(const std_msgs::msg::Bool::SharedPtr msg){
  if (msg->data){
    if (!this->predictor_00->is_calibrating_mag)
      this->predictor_00->calibrate_magnetometer();
    if (this->p1_params.use_predictor and !this->predictor_01->is_calibrating_mag)
      this->predictor_01->calibrate_magnetometer();
  }
} // mag_calib_sub_callback

void AttitudeEstimator::gyro_calib_sub_callback(const std_msgs::msg::Bool::SharedPtr msg){
  if (msg->data){
    if (!this->predictor_00->is_calibrating_gyro)
      this->predictor_00->calibrate_gyroscope();
    if (this->p1_params.use_predictor and !this->predictor_01->is_calibrating_gyro)
      this->predictor_01->calibrate_gyroscope();
  }
} // gyro_calib_sub_callback

void AttitudeEstimator::declare_params(rclcpp::Node * node){
  // general params
  node->declare_parameter("sensor_update_hz");
  node->declare_parameter("state_update_hz");
  // predictor 0 params
  node->declare_parameter("predictor.0.name");
  node->declare_parameter("predictor.0.dev_addr");
  node->declare_parameter("predictor.0.dev_port");
  node->declare_parameter("predictor.0.sensor_dir");
  // predictor 0 gyro params
  node->declare_parameter("predictor.0.gyro.use");
  node->declare_parameter("predictor.0.gyro.hz");
  node->declare_parameter("predictor.0.gyro.max");
  node->declare_parameter("predictor.0.gyro.gain");
  node->declare_parameter("predictor.0.gyro.angular_vel_gain");
  // predictor 0 acc params
  node->declare_parameter("predictor.0.acc.use");
  node->declare_parameter("predictor.0.acc.hz");
  node->declare_parameter("predictor.0.acc.max");
  node->declare_parameter("predictor.0.acc.gain");
  // predictor 0 mag params
  node->declare_parameter("predictor.0.mag.use");
  node->declare_parameter("predictor.0.mag.hz");
  node->declare_parameter("predictor.0.mag.gain");
  // predictor 1 params
  node->declare_parameter("predictor.1.use");
  node->declare_parameter("predictor.1.name");
  node->declare_parameter("predictor.1.dev_addr");
  node->declare_parameter("predictor.1.dev_port");
  node->declare_parameter("predictor.1.sensor_dir");
  // predictor 1 gyro params
  node->declare_parameter("predictor.1.gyro.use");
  node->declare_parameter("predictor.1.gyro.hz");
  node->declare_parameter("predictor.1.gyro.max");
  node->declare_parameter("predictor.1.gyro.gain");
  node->declare_parameter("predictor.1.gyro.angular_vel_gain");
  // predictor 1 acc params
  node->declare_parameter("predictor.1.acc.use");
  node->declare_parameter("predictor.1.acc.hz");
  node->declare_parameter("predictor.1.acc.max");
  node->declare_parameter("predictor.1.acc.gain");
  // predictor 1 mag params
  node->declare_parameter("predictor.1.mag.use");
  node->declare_parameter("predictor.1.mag.hz");
  node->declare_parameter("predictor.1.mag.gain");
} // declare params

void AttitudeEstimator::get_params(rclcpp::Node * node){
  // get optional general parameters (update hz)
  node->get_parameter_or("sensor_update_hz", this->sensor_update_hz, 50.0f);
  node->get_parameter_or("state_update_hz", this->state_update_hz, 100.0f);
  // get predictor 0 params
  this->p0_params.use_predictor = true;
  node->get_parameter_or("predictor.0.name", this->p0_params.name, def_params.name);
  node->get_parameter_or("predictor.0.dev_addr", this->p0_params.dev_addr, def_params.dev_addr);
  node->get_parameter_or("predictor.0.dev_port", this->p0_params.dev_port, def_params.dev_port);
  node->get_parameter_or("predictor.0.sensor_dir", this->p0_params.sensor_dir, def_params.sensor_dir);
  // predictor 0 gyro params
  node->get_parameter_or("predictor.0.gyro.use", this->p0_params.use_gyro, def_params.use_gyro);
  if (this->p0_params.use_gyro){
    node->get_parameter_or("predictor.0.gyro.hz", this->p0_params.gyro_hz, def_params.gyro_hz);
    node->get_parameter_or("predictor.0.gyro.gain", this->p0_params.gyro_gain, def_params.gyro_gain);
    node->get_parameter_or("predictor.0.gyro.angular_vel_gain", this->p0_params.gyro_vel_gain, 
                           def_params.gyro_vel_gain);
    node->get_parameter_or("predictor.0.gyro.max", this->p0_params.gyro_max, def_params.gyro_max);
  }
  // predictor 0 acc params
  node->get_parameter_or("predictor.0.acc.use", this->p0_params.use_acc, def_params.use_acc);
  if (this->p0_params.use_acc){
    node->get_parameter_or("predictor.0.acc.hz", this->p0_params.acc_hz, def_params.acc_hz);
    node->get_parameter_or("predictor.0.acc.gain", this->p0_params.acc_gain, def_params.acc_gain);
    node->get_parameter_or("predictor.0.acc.max", this->p0_params.acc_max, def_params.acc_max);
  }
  // predictor 0 mag params
  node->get_parameter_or("predictor.0.mag.use", this->p0_params.use_mag, def_params.use_mag);
  if (this->p0_params.use_mag){
    node->get_parameter_or("predictor.0.mag.hz", this->p0_params.mag_hz, def_params.mag_hz);
    node->get_parameter_or("predictor.0.mag.gain", this->p0_params.mag_gain, def_params.mag_gain);
  }
  // get predictor 1 params
  node->get_parameter_or("predictor.1.use", this->p1_params.use_predictor, def_params.use_predictor);
  if (this->p1_params.use_predictor){
    node->get_parameter_or("predictor.1.name", this->p1_params.name, def_params.name);
    node->get_parameter_or("predictor.1.dev_addr", this->p1_params.dev_addr, def_params.dev_addr);
    node->get_parameter_or("predictor.1.dev_port", this->p1_params.dev_port, def_params.dev_port);
    node->get_parameter_or("predictor.1.sensor_dir", this->p1_params.sensor_dir, def_params.sensor_dir);
    // predictor 0 gyro params
    node->get_parameter_or("predictor.1.gyro.use", this->p1_params.use_gyro, def_params.use_gyro);
    if (this->p1_params.use_gyro){
      node->get_parameter_or("predictor.1.gyro.hz", this->p1_params.gyro_hz, def_params.gyro_hz);
      node->get_parameter_or("predictor.1.gyro.gain", this->p1_params.gyro_gain, def_params.gyro_gain);
      node->get_parameter_or("predictor.1.gyro.max", this->p1_params.gyro_max, def_params.gyro_max);
      node->get_parameter_or("predictor.1.gyro.angular_vel_gain", this->p0_params.gyro_vel_gain, 
                             def_params.gyro_vel_gain);
    }
    // predictor 1 acc params
    node->get_parameter_or("predictor.1.acc.use", this->p1_params.use_acc, def_params.use_acc);
    if (this->p1_params.use_acc){
      node->get_parameter_or("predictor.1.acc.hz", this->p1_params.acc_hz, def_params.acc_hz);
      node->get_parameter_or("predictor.1.acc.gain", this->p1_params.acc_gain, def_params.acc_gain);
      node->get_parameter_or("predictor.1.acc.max", this->p1_params.acc_max, def_params.acc_max);
    }
    // predictor 1 mag params
    node->get_parameter_or("predictor.1.mag.use", this->p1_params.use_mag, def_params.use_mag);
    if (this->p1_params.use_mag){
      node->get_parameter_or("predictor.1.mag.hz", this->p1_params.mag_hz, def_params.mag_hz);
      node->get_parameter_or("predictor.1.mag.gain", this->p1_params.mag_gain, def_params.mag_gain);
    }
  }
  
  p0_params.print_params();
  p1_params.print_params();
} // void get_params

void AttitudeEstimator::check_params(){
  if (p1_params.use_predictor == true){
    // check for same name
    if (p1_params.name == p0_params.name){
      std::cout << "Initialization failed, identical sensor name detected\n";
      rclcpp::shutdown();
    } else if (p1_params.dev_addr == p0_params.dev_addr and p1_params.dev_port == p0_params.dev_port) {
      std::cout << "Initialization failed, reading identical sensor from identical port\n";
      rclcpp::shutdown();
    } else if (p1_params.dev_port == p0_params.dev_port and (p1_params.use_mag and p0_params.use_mag)){
      std::cout << "Initialization failed, using multiple magnetometer in single I2C channel\n";
      rclcpp::shutdown();
    } else {
      std::cout << "Dual predictor initialization successful\n";
    }
  } else {
    std::cout << "Single predictor initialization successful\n";
  }
} // check_params

std::chrono::time_point<std::chrono::system_clock> AttitudeEstimator::get_system_time(){
  return std::chrono::system_clock::now();
} // get_system_time

float AttitudeEstimator::get_time_diff(auto end_time, auto start_time){
  auto us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
  return (float(us/1e6));
} // get time diff

} // namespace navatics_attitude_estimator
