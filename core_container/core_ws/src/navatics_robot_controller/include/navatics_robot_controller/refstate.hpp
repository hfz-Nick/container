#ifndef NAVATICSROBOTCONTROLLER_REFSTATE_H
#define NAVATICSROBOTCONTROLLER_REFSTATE_H

#include <Eigen/Eigen>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace navatics_robot_controller{

class RefState{
  /*
  This class is designed to:
  1. Receive user input (from joy or teleop) and convert them into understandable reference
     states of the robot
  2. Update the ref_position and ref_orientation of the robot from the given commands
  3. Assumptions:
     - Robot reference orientation is always flat:
       # only Z component of the angular velocity input will be used to update ref_orientation
     - You can activate or deactivate the positioning system only at the initialization
  4. Mode:
     - No positioning system:
       # only the Z component of the velocity input will be used to update the ref_position vector
       # the X and Y component is taken as a user input in body frame. Assuming the robot always flat,
         it is not necessary to transform the input
     - Operating with positioning system:
       # user_command vector will not be calculated
       # all velocity vectors are taken as position and used to calculate ref_position vector
  */

  public:
  RefState(rclcpp::Node * node, Eigen::Vector3f max_velocity, float refstate_update_hz = 50.0, 
           bool is_no_positioning = true);
  
  // initialize
  void initialize(Eigen::Vector3f ref_position, Eigen::Vector3f ref_velocity,
                  Eigen::Quaternionf orientation, Eigen::Vector3f angular_velocity,
                  bool is_no_positioning = true);

  // get
  bool initialized();
  Eigen::Vector3f get_ref_position();
  Eigen::Vector3f get_ref_velocity();
  Eigen::Quaternionf get_ref_orientation();
  Eigen::Vector3f get_ref_angular_velocity();
  Eigen::Vector3f get_user_command();
  
  
  // update
  void update_ref_position(Eigen::Vector3f ref_velocity, float dt);
  void update_ref_velocity(Eigen::Vector3f ref_velocity);
  void update_ref_orientation(Eigen::Vector3f ref_angular_velocity, float dt);
  void update_ref_angular_velocity(Eigen::Vector3f ref_angular_velocity);

  private:
  float refstate_update_hz = 50.0f;
 
  // flags
  bool is_initialized = false;
  bool is_no_positioning = true;
  // storage variables
  Eigen::Vector3f ref_position;
  Eigen::Vector3f ref_velocity;
  Eigen::Vector3f max_velocity;
  Eigen::Quaternionf ref_orientation;
  Eigen::Vector3f ref_angular_velocity;
  Eigen::Vector3f user_command;

  // subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub;
  void cmd_sub_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // timer
  rclcpp::TimerBase::SharedPtr refstate_timer;
  void refstate_timer_callback();
  int refstate_pub_counter = 0;

  // publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher;

}; // class RefState


} // namespace navatics_robot_controller

#endif // NAVATICSROBOTCONTROLLER_REFSTATE_H
