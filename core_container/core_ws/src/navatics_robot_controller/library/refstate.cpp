#include "navatics_robot_controller/refstate.hpp"

namespace navatics_robot_controller{

RefState::RefState(rclcpp::Node * node, Eigen::Vector3f max_velocity, float refstate_update_hz, 
                   bool is_no_positioning){
  this->ref_position << 0,0,0;
  this->ref_velocity << 0,0,0;
  this->ref_orientation = Eigen::Quaternionf(1,0,0,0);
  this->ref_angular_velocity << 0,0,0;
  this->max_velocity = max_velocity;
  
  this->refstate_update_hz = refstate_update_hz;

  this->odometry_publisher = node->create_publisher<nav_msgs::msg::Odometry>("refstate/odometry", 100);
  using namespace std::placeholders;
  this->cmd_sub = node->create_subscription<geometry_msgs::msg::Twist> ("cmd_vel", 10,
                    std::bind(&RefState::cmd_sub_callback, this, _1));
  std::chrono::milliseconds update_duration_ms((int)(1000/this->refstate_update_hz));
  this->refstate_timer = node->create_wall_timer( update_duration_ms,
                          std::bind(&RefState::refstate_timer_callback, this) );
  this->refstate_pub_counter = 0;
  this->is_no_positioning = is_no_positioning;
} // constructor

void RefState::refstate_timer_callback(){
  if (this->initialized()){
    float dt = 1/this->refstate_update_hz;
    this->update_ref_position(this->ref_velocity, dt);
    this->update_ref_orientation(this->ref_angular_velocity, dt);
    if (this->refstate_pub_counter % 5 == 0){
      auto msg = nav_msgs::msg::Odometry();
      msg.pose.pose.position.x = this->ref_position[0];
      msg.pose.pose.position.y = this->ref_position[1];
      msg.pose.pose.position.z = this->ref_position[2];
      msg.pose.pose.orientation.w = this->ref_orientation.w();
      msg.pose.pose.orientation.x = this->ref_orientation.x();
      msg.pose.pose.orientation.y = this->ref_orientation.y();
      msg.pose.pose.orientation.z = this->ref_orientation.z();
      msg.twist.twist.linear.x = 0;
      msg.twist.twist.linear.y = 0;
      msg.twist.twist.linear.z = this->ref_velocity[2];
      msg.twist.twist.angular.x = this->ref_angular_velocity[0];
      msg.twist.twist.angular.y = this->ref_angular_velocity[1];
      msg.twist.twist.angular.z = this->ref_angular_velocity[2];
      this->odometry_publisher->publish(msg);
      this->refstate_pub_counter = 0;
    }
    this->refstate_pub_counter += 1;
  } 
}

void RefState::cmd_sub_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
  if (this->initialized()){
    Eigen::Vector3f ref_ff_velocity, ref_angular_velocity;
    ref_ff_velocity << msg->linear.x, msg->linear.y, msg->linear.z;
    ref_angular_velocity << msg->angular.x, msg->angular.y, msg->angular.z;
    this->update_ref_velocity(ref_ff_velocity);
    this->update_ref_angular_velocity(ref_angular_velocity);
  }
} // cmd_sub_callback

bool RefState::initialized(){
  return this->is_initialized;
} // initialized()

Eigen::Vector3f RefState::get_ref_position(){
  return this->ref_position;
} // get_ref_position

Eigen::Vector3f RefState::get_ref_velocity(){
  return this->ref_velocity;
} // get_ref_velocity

Eigen::Vector3f RefState::get_ref_angular_velocity(){
  return this->ref_angular_velocity;
} // get_ref_angular_velocity

Eigen::Quaternionf RefState::get_ref_orientation(){
  return this->ref_orientation;
} // get_ref_orientation

Eigen::Vector3f RefState::get_user_command(){
  return this->user_command;
} // get_user_command

void RefState::update_ref_velocity(Eigen::Vector3f ref_velocity){
  if (this->is_no_positioning){
    this->ref_velocity << 0, 0, ref_velocity[2];
    this->user_command << ref_velocity[0], ref_velocity[1], 0;
  } else {
    this->ref_velocity = ref_velocity;
    this->user_command << 0,0,0;
  }
} // update_ref_velocity

void RefState::update_ref_angular_velocity(Eigen::Vector3f ref_angular_velocity){
  this->ref_angular_velocity << 0, 0, ref_angular_velocity[2];
} // update_refangular_velocity

void RefState::update_ref_position(Eigen::Vector3f ref_velocity, float dt){
  Eigen::Vector3f v (0, 0, ref_velocity[2]*this->max_velocity[2]); // mode: no positioning
  this->ref_position += v*dt;
  this->ref_position[2] = (this->ref_position(2) < -1.0f) ? -1.0f : this->ref_position(2);
} // update_ref_position

void RefState::update_ref_orientation(Eigen::Vector3f ref_angular_velocity, float dt){
  // mode: ref state is always flat
  float angle = ref_angular_velocity(2)*dt;
  Eigen::AngleAxisf aa(angle, Eigen::Vector3f::UnitZ());
  Eigen::Quaternionf qe(aa);
  this->ref_orientation = qe*this->ref_orientation;
} // update_ref_orientation

void RefState::initialize(Eigen::Vector3f ref_position, Eigen::Vector3f ref_velocity,
                          Eigen::Quaternionf orientation, Eigen::Vector3f angular_velocity,
                          bool is_no_positioning)
{
  if ( !this->initialized() ){
    this->ref_position = ref_position;
    this->ref_position[2] = (this->ref_position(2) < 0.0f) ? 0.0f : this->ref_position(2);
    this->ref_velocity = Eigen::Vector3f(0,0,0);
    this->ref_orientation = ref_orientation.normalized();
    this->ref_angular_velocity = Eigen::Vector3f(0,0,0);
    this->user_command << 0,0,0;
    this->is_no_positioning = is_no_positioning;
    this->is_initialized = true;
  }
} // initialize

} // namespace navatics_robot_controller
