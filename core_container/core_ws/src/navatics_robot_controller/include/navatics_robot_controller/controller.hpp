#ifndef NAVATICSROBOTCONTROLLER_CONTROLLER_H
#define NAVATICSROBOTCONTROLLER_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "navatics_robot_controller/position.hpp"
#include "navatics_robot_controller/orientation.hpp"
#include "navatics_robot_controller/refstate.hpp"
#include "navatics_robot_controller/thrusters.hpp"

#include <string>
#include <iostream>
#include <algorithm>

#include <Eigen/Eigen>

namespace navatics_robot_controller{

class ControllerParams{
  /*
  This class is designed to:
  1. Subscribe to states and handle the reference states
  2. Pass states and reference states to controllers
  3. Pass control input into RPM controller
  4. Publsih RPM to CAN Bus messages
  5. Load important parameters for controller initialization
  6. Overall wrapper of the ROV control function
  */

  public:
  ControllerParams(){}

  std::string controller_name;
  Eigen::Vector3f p_gain;
  Eigen::Vector3f d_gain;
  bool use_compensation = true;

  void initialize(std::string name){
    this->controller_name = name;
    this->p_gain << 0,0,0;
    this->d_gain << 0,0,0;
  }
  
  void print(){
    std::cout << "\n" << this->controller_name << "\n";
    std::cout << "P gain: " << p_gain.transpose() << "\n";
    std::cout << "D gain: " << d_gain.transpose() << "\n";
    std::cout << "use compensation: " << this->use_compensation << "\n";
  }

}; //class ControllerParams

class ControllerBiasParams{
  public:
  ControllerBiasParams(){}
  
  float weight = 0.0f;
  float buoyancy = 0.0f;
  Eigen::Vector3f center_of_gravity;
  Eigen::Vector3f center_of_buoyancy;

  void print(){
    std::cout << "\nbias\n";
    std::cout << "weight : " << weight << " center: " << center_of_gravity.transpose() << "\n";
    std::cout << "buoyancy : " << buoyancy << " center: " << center_of_buoyancy.transpose() << "\n";
  }
}; // class ControllerBiasParams

class Controller{
  
  public:
  Controller(rclcpp::Node * node);

  private:
  // update rate
  float controller_update_hz =100.0f;
  float refstate_update_hz = 50.0f;
  float rpm_update_hz =100.0f;

  // parameters handler
  void declare_params(rclcpp::Node * node);
  void get_params(rclcpp::Node * node);
  Eigen::Vector3f get_float_vector(rclcpp::Parameter p);
  Eigen::Vector3f max_cmd_force;
  Eigen::Vector3f max_velocity;
  ControllerParams position_params;
  ControllerParams orientation_params;
  ControllerBiasParams bias_params;

  // controllers
  Position *position_controller;
  Orientation *orientation_controller;
  Thrusters *thrusters_controller;
  Eigen::VectorXi rpm;

  // subscribers
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr orientation_sub;
  void orientation_sub_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr position_sub;
  void position_sub_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angular_velocity_sub;
  void angular_velocity_sub_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr velocity_sub;
  void velocity_sub_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr orientation_flag_sub;
  void orientation_flag_sub_callback(const std_msgs::msg::UInt8::SharedPtr msg);

  // timers
  rclcpp::TimerBase::SharedPtr controller_timer;
  void controller_timer_callback();
  rclcpp::TimerBase::SharedPtr rpm_timer;
  void rpm_timer_callback();
  
  // states
  Eigen::Quaternionf orientation;
  Eigen::Vector3f angular_velocity;
  Eigen::Vector3f position;
  Eigen::Vector3f velocity;
  // ref states
  int orientation_flag = 0;
  RefState *ref;
  // user command
  Eigen::Vector3f user_cmd;
  // initializer
  void initialize_states();
  // state flags
  bool is_orientation_initialized = false;
  bool is_position_initialized = false;
  bool is_controller_initialized = false;

}; // class controller

} // namespace navatics_robot_controller

#endif // NAVATICSROBOTCONTROLLER_CONTROLLER_H
