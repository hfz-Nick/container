#include "navatics_server/mavlink_state_pub.hpp"

namespace navatics_server{

MAVLinkStatePub::MAVLinkStatePub(rclcpp::Node * node){
  using namespace std::placeholders;
  this->position_sub = node->create_subscription<geometry_msgs::msg::Vector3>("state/position", 
                               1000, std::bind(&MAVLinkStatePub::position_cb, this, _1));
  this->velocity_sub = node->create_subscription<geometry_msgs::msg::Vector3>("state/velocity",
                               1000, std::bind(&MAVLinkStatePub::velocity_cb, this, _1));
  this->orientation_sub = node->create_subscription<geometry_msgs::msg::Quaternion>("state/orientation",
                               1000, std::bind(&MAVLinkStatePub::orientation_cb, this, _1));
  this->angular_velocity_sub = node->create_subscription<geometry_msgs::msg::Vector3>
                               ("state/angular_velocity", 1000, 
                               std::bind(&MAVLinkStatePub::angular_velocity_cb, this, _1));
  this->refstate_sub = node->create_subscription<nav_msgs::msg::Odometry>
                       ("refstate/odometry", 1000,
                        std::bind(&MAVLinkStatePub::ref_odometry_cb, this, _1));
  this->mpu_calib_sub = node->create_subscription<std_msgs::msg::Bool>("mpu/calibrated", 100,
                          std::bind(&MAVLinkStatePub::mpu_calib_cb, this, _1));
  this->thruster_active_sub = node->create_subscription<std_msgs::msg::Bool>
                              ( "controller/rpm/active", 100,
                              std::bind(&MAVLinkStatePub::thruster_active_cb, this, _1));
} // constructor

State MAVLinkStatePub::get_current_state(){
  State s(this->nedx, this->nedy, this->nedz, this->vx, this->vy, this->vz,
          this->qw, this->qx, this->qy, this->qz, this->wx, this->wy, this->wz);
  return s;
} // get_current_state

State MAVLinkStatePub::get_current_refstate(){
  State s(this->r_nedx, this->r_nedy, this->r_nedz, this->r_vx, this->r_vy, this->r_vz,
          this->r_qw, this->r_qx, this->r_qy, this->r_qz, this->r_wx, this->r_wy, this->r_wz);
  return s;
} // get_current_refstate

void MAVLinkStatePub::mpu_calib_cb(const std_msgs::msg::Bool::SharedPtr msg){
  this->is_mpu_calibrated = msg->data;
} // mpu_calib_cb

void MAVLinkStatePub::thruster_active_cb(const std_msgs::msg::Bool::SharedPtr msg){
  this->is_thruster_active = msg->data;
} // thrusters_active_cb

void MAVLinkStatePub::ref_odometry_cb(const nav_msgs::msg::Odometry::SharedPtr msg){
  if (!this->is_ref_odometry_initialized)
    this->is_ref_odometry_initialized = true;
  this->r_nedx = msg->pose.pose.position.x;
  this->r_nedy = msg->pose.pose.position.y;
  this->r_nedz = msg->pose.pose.position.z;
  this->r_qw = msg->pose.pose.orientation.w;
  this->r_qx = msg->pose.pose.orientation.x;
  this->r_qy = msg->pose.pose.orientation.y;
  this->r_qz = msg->pose.pose.orientation.z;
  this->r_vx = msg->twist.twist.linear.x;
  this->r_vy = msg->twist.twist.linear.y;
  this->r_vz = msg->twist.twist.linear.z;
  this->r_wx = msg->twist.twist.angular.x;
  this->r_wy = msg->twist.twist.angular.y;
  this->r_wz = msg->twist.twist.angular.z;
} // position_cb

void MAVLinkStatePub::position_cb(const geometry_msgs::msg::Vector3::SharedPtr msg){
  if (!this->is_position_initialized)
    this->is_position_initialized = true;
  this->nedx = msg->x;
  this->nedy = msg->y;
  this->nedz = msg->z;
} // position_cb

void MAVLinkStatePub::velocity_cb(const geometry_msgs::msg::Vector3::SharedPtr msg){
  this->vx = msg->x;
  this->vy = msg->y;
  this->vz = msg->z;
} // velocity_cb

void MAVLinkStatePub::orientation_cb(const geometry_msgs::msg::Quaternion::SharedPtr msg){
  if (!this->is_orientation_initialized)
    this->is_orientation_initialized = true;
  this->qw = msg->w;
  this->qx = msg->x;
  this->qy = msg->y;
  this->qz = msg->z;
} // orientation_cb

void MAVLinkStatePub::angular_velocity_cb(const geometry_msgs::msg::Vector3::SharedPtr msg){
  this->wx = msg->x;
  this->wy = msg->y;
  this->wz = msg->z;
} // angular_velocity_cb

} // namespace navatics_server
