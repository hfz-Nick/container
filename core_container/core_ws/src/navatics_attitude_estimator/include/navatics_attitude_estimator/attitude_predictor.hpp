#ifndef NAVATICSATTITUDEESTIMATOR_ATTITUDEPREDICTOR_H
#define NAVATICSATTITUDEESTIMATOR_ATTITUDEPREDICTOR_H

#include "navatics_i2c/devices/MPU9150.h"
#include "navatics_attitude_estimator/orientation.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <random>

#include <Eigen/Eigen>

namespace navatics_attitude_estimator{

#define MPU_DEFAULT_ADDR MPU9150_ADDRESS_AD0_LOW
#define MPU_DEFAULT_PORT "/dev/i2c-0"

#define ACC_RANGE_2G   MPU9150_ACCEL_FS_2
#define ACC_RANGE_4G   MPU9150_ACCEL_FS_4
#define ACC_RANGE_8G   MPU9150_ACCEL_FS_8
#define ACC_RANGE_16G  MPU9150_ACCEL_FS_16

#define GYRO_RANGE_250DPS   MPU9150_GYRO_FS_250
#define GYRO_RANGE_500DPS   MPU9150_GYRO_FS_500
#define GYRO_RANGE_1000DPS  MPU9150_GYRO_FS_1000
#define GYRO_RANGE_2000DPS  MPU9150_GYRO_FS_2000

#define MAX_BIT_VAL 32768 // Divider for 16 bit integer
#define MAX_MAG_BIT_VAL 8192 // Divider for 14 bit integer (Magnetometer)
#define G 9.80665 // Gravity constant, for accelerometer conversion
#define PI 3.14
#define DEG_TO_RAD PI/180 // For gyroscope conversion

#define MAG_RANGE 4800.0 // Maximum scale for magnetometer +/- 4800 uT

#define ACC_SELECT 0
#define GYRO_SELECT 1
#define MAG_SELECT 2

class AttitudePredictor{
  /*
  This class is designed to:
  1. Create loop that drives data-reading from I2C devices and
     update sensor readings to the latest reading for calculation purposes
  2. Create loop that calculates current orientation status and
     send the update signal to the attitude state manager
  3. Each one of this class is responsible for 1 MPU device. If more device is needed,
     simply spawn another copy of this class
  4. Handle calibration routine
  */

  public:
  AttitudePredictor(rclcpp::Node * node,
                    uint8_t mpu_dev_addr = MPU_DEFAULT_ADDR, 
                    std::string mpu_dev_port = MPU_DEFAULT_PORT,
                    std::string sensor_name = "mpu00",
                    std::string sensor_dir = "",
                    bool is_reading_acc = true, uint8_t acc_range = ACC_RANGE_2G,
                    float acc_update_hz = 100.0,
                    bool is_reading_gyro = true, uint8_t gyro_range = GYRO_RANGE_250DPS,
                    float gyro_update_hz = 100.0,
                    bool is_reading_mag = true, float mag_update_hz = 80.0, 
                    bool is_debug = false, bool is_testing_rotation = false);

  // function used for unit tests
  int mpu_test();
  int get_rotation_test(float acc_diff_threshold = 0.1, float mag_diff_threshold = 0.1,
                        float gyro_diff_threshold = 0.1);
  void debug(bool is_debug = true);

  // initialize a state or link state pointer to a state
  void initialize(Orientation *state);
  void link_state(Orientation *state);
  void set_update_weights(float gyro = 0.9, float gyro_angular_velocity = 0.95, float acc = 0.1, 
                          float mag = 0.1);
  
  // calibration functions and flags to be called by wrapper
  // gyroscope
  bool is_calibrating_gyro = false;
  bool is_gyro_calibrated = false;
  void calibrate_gyroscope();
  // magnetometer
  bool is_calibrating_mag = false;
  bool is_mag_calibrated = false;
  void calibrate_magnetometer();

  // publish sensor data
  void publish_sensor_data();

  bool is_mpu_calibrated();
  
  private:
  // MPU object
  MPU9150 mpu;
  uint8_t mpu_dev_addr = 0x68;
  std::string mpu_dev_port = "/dev/i2c-0";
  std::string sensor_name = "mpu00";
  
  // state object
  Orientation *state;

  // sensor data publishers
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr acc_publisher;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gyro_publisher;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr mag_publisher;

  // flags
  bool is_debug = false;
  bool is_reading_acc = true;
  bool is_reading_gyro = true;
  bool is_reading_mag = true;
  
  // timers for reading sensor data
  //acc
  float acc_update_hz = 100.0;
  rclcpp::TimerBase::SharedPtr acc_timer;
  void acc_read_callback();
  //gyro
  float gyro_update_hz = 100.0;
  rclcpp::TimerBase::SharedPtr gyro_timer;
  void gyro_read_callback();
  // mag
  float mag_update_hz = 100.0;
  rclcpp::TimerBase::SharedPtr mag_timer;
  void mag_read_callback();

  // accelerometer and gyroscope ranges
  float acc_range = 2*G;
  float gyro_range = 250.0*DEG_TO_RAD;
  Eigen::Vector3f get_sensor_reading(int sensor_type = GYRO_SELECT);

  // acc, mag, and gyro storage variables
  Eigen::Vector3f update_reading(int16_t buffer[3], float scaler);
  Eigen::Vector3f acc_vector, gyro_vector, mag_vector;
  
  // rotation calculation
  Eigen::Quaternionf get_acc_rotation(Eigen::Vector3f north, Eigen::Vector3f down);
  Eigen::Quaternionf get_mag_rotation(Eigen::Vector3f north, Eigen::Vector3f down);
  Eigen::Quaternionf get_gyro_rotation(Eigen::Vector3f w, Eigen::Quaternionf q, float dt);
  std::chrono::time_point<std::chrono::system_clock> start_time;
  std::chrono::time_point<std::chrono::system_clock> get_system_time();
  float get_time_diff(auto end_time, auto start_time);
  float w_gyro = 0.9;
  float w_angular_vel_gyro = 0.95;
  float w_acc = 0.1;
  float w_mag = 0.1;

  // calibration handler
  // write and load
  void load_calibration_file(std::string filename, int sensor_type = GYRO_SELECT);
  void write_calibration_file(std::string filename, Eigen::Vector3f bias, bool flag=true);
  // magnetometer
  std::vector <float> mx_container;
  std::vector <float> my_container;
  std::vector <float> mz_container;
  std::string mag_calib_filename;
  Eigen::Vector3f mag_bias;
  // magnetometer calibration parameters
  float mag_distance_threshold = 2.0f; // unit: uTesla
  float mag_radius_threshold = 20.0f; // unit: uTesla
  float mag_theta_ref_threshold = 0.2f; // unit: rad
  int mag_min_exit_len = 30; // num of reading needed before exit
  std::vector <float> mag_theta_reference = {0, PI/6, PI/3, PI/2, 2*PI/3, 5*PI/6,                                                                    PI, -PI/6, -PI/3, -PI/2, -2*PI/3, -5*PI/6};
  // magnetometer calibration helper functions
  bool get_mag_data(Eigen::Vector3f vector);
  bool get_mag_exit_status(std::vector <bool> * flags, float theta);
  // magnetometer calibration timeout function
  std::chrono::time_point<std::chrono::system_clock> mag_calib_start_time;
  float mag_calib_timeout_sec = 30;
  
  // gyroscope
  std::vector <Eigen::Vector3f> gyro_vec_container;
  std::string gyro_calib_filename;
  Eigen::Vector3f gyro_bias;
  
  // threshold
  float gyro_bias_threshold = 0.1; // max magnitude of combined 3-axis gyro reading
  
  // gyro calibration helper function
  bool update_gyro_bias();

}; // class AttitudePredictor

} // namespace navatics_attitude_estimator


#endif // NAVATICSATTITUDEESTIMATOR_ATTITUDEPREDICTOR_H
