#ifndef NAVATICSROBOTCONTROLLER_THRUSTERS_H
#define NAVATICSROBOTCONTROLLER_THRUSTERS_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <navatics_msgs/msg/can_bus.hpp>

#include <serial/serial.h>

#include <Eigen/Eigen>

#include <vector>
#include <set>

#define USART_STARTBYTES_0 0xBE
#define USART_STARTBYTES_1 0xEF
#define USART_STOPBYTES_0 0xCA
#define USART_STOPBYTES_1 0xFE

namespace navatics_robot_controller{

class Thrusters{

  public:
  Thrusters(rclcpp::Node * node);

  // parameters handler
  void declare_params(rclcpp::Node * node);
  void get_params(rclcpp::Node * node);

  // calculate rpm
  Eigen::VectorXi calculate_rpm(Eigen::Vector3f force, Eigen::Vector3f moment);

  // publish rpm  
  void publish_rpm(Eigen::VectorXi rpm);

  private:
  // thruster parameters
  Eigen::MatrixXf force_tf_matrix;
  std::vector<int> thrusters_id;
  std::vector<int> rpm_mod;
  std::vector<int64_t> msg_assignment;
  int rpm_msg_len;
  // lookup table parameters
  std::vector<int16_t> rpm_lut;
  std::vector<double> thrust_lut;
  int n_lut;

  // calculation parameters
  int n_thrusters;
  std::set<int> unique_id;
  std::vector < std::vector<int> > id_index_map;
  Eigen::Vector3f max_force, min_force;
  Eigen::Vector3f max_moment, min_moment;
  bool is_thrusters_active = false;
  
  // helper functions
  Eigen::Vector3f saturate(Eigen::Vector3f input, Eigen::Vector3f max_saturation, 
                           Eigen::Vector3f min_saturation);
  Eigen::VectorXf calculate_thrust(Eigen::Vector3f force, Eigen::Vector3f moment);
  int rpm_lookup(float force);

  // subscribers
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rpm_switch_sub;
  void rpm_switch_sub_callback(const std_msgs::msg::Bool::SharedPtr msg);

  // publishers
  // rclcpp::Publisher<navatics_msgs::msg::CanBus>::SharedPtr can_bus_publisher;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_status_publisher;

  // USART based RPM publishers
  serial::Serial serial_device;
  std::string serial_port = "/dev/ttyUSB0";
  int serial_baudrate = 115200;
  int serial_timeout_ms = 1000;
  int buffer_len;
  
  // timing helper function
  std::chrono::time_point<std::chrono::system_clock> last_status_time;
  std::chrono::time_point<std::chrono::system_clock> get_system_time();
  float get_time_diff(auto end_time, auto start_time);

  // status update
  float status_update_hz = 5; // Hz
}; // class Thrusters

} // namespace navatics_robot_controller

#endif // NAVATICSROBOTCONTROLLER_THRUSTERS_H
