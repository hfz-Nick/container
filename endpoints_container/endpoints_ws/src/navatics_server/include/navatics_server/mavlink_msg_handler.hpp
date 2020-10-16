#ifndef NAVATICSSERVER_MAVLINKMSGHANDLER_H
#define NAVATICSSERVER_MAVLINKMSGHANDLER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <mavlink/common/mavlink.h>

#include "navatics_msgs/msg/can_bus.hpp"

#include <iostream>

#include <chrono>
#include <string>
#include <cstring>

#define THRUSTER_CMD_ID  0xA0
#define THRUSTER_CMD_ON  0X01
#define THRUSTER_CMD_OFF 0xFF
#define PARAM_ID_LEN (16)

typedef union MsgUnion {
  float vaue_f;
  int32_t vaue_i;
} MsgUnion;

namespace navatics_server{

// utility functions
std::vector<std::string> split(std::string str, std::string pattern);

// Message Acknowledgement Class for Return Value
class MsgAck {
  
  public:
  MsgAck(){
    this->enable_ack = false;
    this->enable_msg = false;
    mavlink_message_t _msg;
    this->msg = _msg;

    mavlink_message_t _ack;
    this->ack = _ack;
  }
  
  MsgAck(mavlink_message_t ack){
    this->enable_ack = true;
    this->ack = ack;
  }
  MsgAck(mavlink_message_t ack, mavlink_message_t msg){
    this->enable_ack = true;
    this->enable_msg = true;
    this->ack = ack;
    this->msg = msg;
  }

  mavlink_message_t msg;
  mavlink_message_t ack;
  bool enable_ack = false;
  bool enable_msg = false;
}; // class MsgAcknowledgment

class MAVLinkMsgHandler {
  
  public:
  MAVLinkMsgHandler(rclcpp::Node * node, std::string cmd_vel_topic_name, std::string rpm_sw_topic_name,
                    uint8_t system_id, uint8_t component_id);
  
  MsgAck parse(mavlink_message_t msg);
  void publish_connected();
  void publish_disconnected();
 
  private:
  uint8_t system_id;
  uint8_t component_id;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rpm_switch_publisher;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
  rclcpp::Publisher<navatics_msgs::msg::CanBus>::SharedPtr can_publisher;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr tilt_flag_publisher;

  // ID validation function
  bool is_sys_id_valid(uint8_t msg_target_sys_id){
    return this->system_id == msg_target_sys_id;
  }

  bool is_comp_id_valid(uint8_t msg_target_comp_id){
    return this->component_id == msg_target_comp_id;
  }
 
}; // MAVLinkMsgHandler

} // namespace navatics_server

#endif // NAVATICSSERVER_MAVLINKMSGHANDLER_H
