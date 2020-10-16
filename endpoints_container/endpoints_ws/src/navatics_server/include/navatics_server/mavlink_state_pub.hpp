#ifndef NAVATICSSERVER_MAVLINKSTATEPUB_H
#define NAVATICSSERVER_MAVLINKSTATEPUB_H

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace navatics_server{

class State{
  public:
  State(float x, float y, float z, float vx, float vy, float vz, 
        float qw, float qx, float qy, float qz, float wx, float wy, float wz)
  {
    this->nedx = x;  this->nedy = y; this->nedz = z;  
    this->vx = vx;  this->vy = vy; this->vz = vz;  
    this->qw = qw; this->qx = qx;  this->qy = qy; this->qz = qz;  
    this->wx = wx;  this->wy = wy; this->wz = wz;  
  }
  
  // state storage variables
  float nedx, nedy, nedz;
  float vx, vy, vz;
  float qw, qx, qy, qz;
  float wx, wy, wz;
  
}; // class State

class MAVLinkStatePub{

  public:
  MAVLinkStatePub(rclcpp::Node * node);
  
  State get_current_state();

  State get_current_refstate();

  bool is_mpu_calibrated = false;
  bool is_thruster_active = false;

  private:
  // subscribe to position, quaternion, and velocity
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr position_sub;
  void position_cb(const geometry_msgs::msg::Vector3::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr velocity_sub;
  void velocity_cb(const geometry_msgs::msg::Vector3::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr orientation_sub;
  void orientation_cb(const geometry_msgs::msg::Quaternion::SharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angular_velocity_sub;
  void angular_velocity_cb(const geometry_msgs::msg::Vector3::SharedPtr msg);
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr refstate_sub;
  void ref_odometry_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
  
  // subscribe to motor status and calibration status
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mpu_calib_sub;
  void mpu_calib_cb(const std_msgs::msg::Bool::SharedPtr msg);
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr thruster_active_sub;
  void thruster_active_cb(const std_msgs::msg::Bool::SharedPtr msg);

  // state storage variables
  float nedx, nedy, nedz;
  float vx, vy, vz;
  float qw, qx, qy, qz;
  float wx, wy, wz;
  // for reference states
  float r_nedx, r_nedy, r_nedz;
  float r_qw, r_qx, r_qy, r_qz;
  float r_vx, r_vy, r_vz;
  float r_wx, r_wy, r_wz;
  // flags
  bool is_position_initialized = false;
  bool is_orientation_initialized = false;
  bool is_ref_odometry_initialized = false;

}; // class MAVLinkStatePub

} // namespace navatics_server

#endif // NAVATICSSERVER_MAVLINKSTATEPUB_H
