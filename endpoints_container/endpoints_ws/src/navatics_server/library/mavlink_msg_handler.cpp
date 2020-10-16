#include "navatics_server/mavlink_msg_handler.hpp"
#include <iostream>

namespace navatics_server{

std::vector<std::string> split(std::string str,std::string pattern) {
  std::string::size_type pos;
  std::vector<std::string> result;
  str+=pattern;
  int size=str.size();
  for(int i=0; i<size; i++) {
    pos=str.find(pattern,i);
    if(pos<size) {
      std::string s=str.substr(i,pos-i);
      result.push_back(s);
      i=pos+pattern.size()-1;
    }
  }
  return result;
}


MAVLinkMsgHandler::MAVLinkMsgHandler(rclcpp::Node * node, std::string cmd_vel_topic_name,
                                     std::string rpm_sw_topic_name, uint8_t system_id,
                                     uint8_t component_id)
{
  this->system_id = system_id;
  this->component_id = component_id;
  this->cmd_vel_publisher = node->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_name, 100);
  this->can_publisher = node->create_publisher<navatics_msgs::msg::CanBus>("/can/transmit", 100);
  this->rpm_switch_publisher = node->create_publisher<std_msgs::msg::Bool>(rpm_sw_topic_name, 10);
  this->tilt_flag_publisher = node->create_publisher<std_msgs::msg::UInt8>
                                ("/state/orientation/flag", 10);
} // constructor

void MAVLinkMsgHandler::publish_connected(){
  auto message = navatics_msgs::msg::CanBus();
  std::vector <uint8_t> data_vec;
  data_vec.push_back(0);
  message.std_id = 0x0001;
  message.length = 1;
  message.data = data_vec;
  this->can_publisher->publish(message);
}

void MAVLinkMsgHandler::publish_disconnected(){
  auto rpm = std_msgs::msg::Bool();
  rpm.data = false;
  this->rpm_switch_publisher->publish(rpm);
  
  auto message = geometry_msgs::msg::Twist();
  message.linear.x = 0.0;
  message.linear.y = 0.0;
  message.linear.z = 0.0;
  message.angular.z = 0.0;
  this->cmd_vel_publisher->publish(message);
}

MsgAck MAVLinkMsgHandler::parse(mavlink_message_t msg){
  if (msg.msgid == MAVLINK_MSG_ID_SERIAL_CONTROL){ // control msg
    mavlink_serial_control_t serial_control;
    mavlink_msg_serial_control_decode(&msg, &serial_control);
    std::string data = std::string((char *)serial_control.data);
    std::string delimiter = "|";
    std::string token = data.substr(0, data.find(delimiter));
    if (token.compare("manual") == 0) {
      data = data.substr(7);
      std::vector<std::string> data_ = split(data, ":");
      int i = 0;
      int ctrl[4] = {0};
      for (auto str : data_) {
        if (i > 3) break;
        int num = std::stoi(str);
        ctrl[i] = num;
        i++;
        // generate ROS twist message
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = (float)ctrl[0]/1000;
        message.linear.y = (float)ctrl[1]/1000;
        message.linear.z = (float)ctrl[2]/1000;
        message.angular.z = (float)ctrl[3]/1000;
        this->cmd_vel_publisher->publish(message);
      } 
    } else if (token.compare("tilt") == 0){
      data = data.substr(5);
      uint8_t tilt_flag = std::stoi(data);
      if (tilt_flag > 5)
        tilt_flag = 0;
      auto message = std_msgs::msg::UInt8();
      message.data = tilt_flag;
      this->tilt_flag_publisher->publish(message);
    } else if (token.compare("led") == 0){
      std::cout << "LED " << data << "\n";
      data = data.substr(4);
      uint8_t led_data = std::stoi(data);
      std::vector <uint8_t> led_data_vec;
      led_data_vec.push_back(led_data);
      auto message = navatics_msgs::msg::CanBus();
      message.std_id = 0x000A;
      message.length = 1;
      message.data = led_data_vec;
      this->can_publisher->publish(message);
    }
  } else if (msg.msgid == MAVLINK_MSG_ID_TIMESYNC) { // TimeSync message
    mavlink_timesync_t timesync;
    mavlink_msg_timesync_decode(&msg, &timesync);
    if (timesync.tc1 == 0) {
      mavlink_message_t rsp;
      auto now = std::chrono::system_clock::now();
      int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds> \
                       (now.time_since_epoch()).count();
      mavlink_msg_timesync_pack(this->system_id, this->component_id, &rsp, now_ns, timesync.ts1);
      MsgAck ret(rsp);
      return ret;
    }
  } else if(msg.msgid == MAVLINK_MSG_ID_PARAM_REQUEST_READ) { // param request msg
    mavlink_param_request_read_t param_request;
    mavlink_msg_param_request_read_decode(&msg, &param_request);
    // char param_id_long_enough[PARAM_ID_LEN + 1] = {};
    // std::memcpy(param_id_long_enough, param_request.param_id, PARAM_ID_LEN);
    MsgUnion msgParam;
    std::string param_id = std::string(param_request.param_id);
    if (param_id.compare("CAL_GYRO0_ID") == 0){
      //TODO::get GYRO health status
      msgParam.vaue_i = 1;
      mavlink_message_t rsp;
      mavlink_msg_param_value_pack(this->system_id, this->component_id, &rsp,
                               param_request.param_id, msgParam.vaue_f, MAV_PARAM_TYPE_INT32, 0, 0);
      MsgAck ret(rsp);
      return ret;
    } else if (param_id.compare("CAL_ACC0_ID") == 0) {
      //TODO::get ACC health status (adaptive cruise control)
      msgParam.vaue_i = 1;
      mavlink_message_t rsp;
      mavlink_msg_param_value_pack(this->system_id, this->component_id, &rsp,
                               param_request.param_id, msgParam.vaue_f, MAV_PARAM_TYPE_INT32, 0, 0);
      MsgAck ret(rsp);
      return ret;
    } else if (param_id.compare("CAL_MAG0_ID") == 0) {
      //TODO::get Compass health status
      msgParam.vaue_i = 1;
      mavlink_message_t rsp;
      mavlink_msg_param_value_pack(this->system_id, this->component_id, &rsp,
                               param_request.param_id, msgParam.vaue_f, MAV_PARAM_TYPE_INT32, 0, 0);
      MsgAck ret(rsp);
      return ret;
    } else if (param_id.compare("SYS_HITL") == 0) {
      msgParam.vaue_i = 0; // Hardware In The Loop
      mavlink_message_t rsp;
      mavlink_msg_param_value_pack(this->system_id, this->component_id, &rsp,
                               param_request.param_id, msgParam.vaue_f, MAV_PARAM_TYPE_INT32, 0, 0);
      MsgAck ret(rsp);
      return ret;
    }
  } else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) { // cmd long message
    mavlink_command_long_t cmd;
    mavlink_msg_command_long_decode(&msg, &cmd);
    if (this->is_sys_id_valid(cmd.target_system) and this->is_comp_id_valid(cmd.target_component)) {
      if (cmd.command == MAV_CMD_COMPONENT_ARM_DISARM)
      {
        auto message = std_msgs::msg::Bool();
        message.data = false;
        if (cmd.param1 == 1){
          message.data = true;
        }
        this->rpm_switch_publisher->publish(message);
        mavlink_message_t ack;
        mavlink_msg_command_ack_pack(this->system_id, this->component_id, &ack, cmd.command, 
                                     0, 0, 0, 0, 0);
        MsgAck ret(ack);
        std::cout << "Received ARM_DISARM " << (int)cmd.param1 << "\n";
        return ret;
      } else if (cmd.command == MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES) { // autopilot status msg
        mavlink_message_t ack;
        mavlink_msg_command_ack_pack(this->system_id, this->component_id, &ack, cmd.command, 
                                     0, 0, 0, 0, 0);
        mavlink_message_t rsp;
        uint8_t flight_custom_version[8] = {0};
        uint8_t middleware_custom_version[8] = {0};
        uint8_t os_custom_version[8] = {0};
        uint8_t uid2[18] = {0};
        // TODO:: get uuid
        uint64_t uid = 111111;
        mavlink_msg_autopilot_version_pack(this->system_id, this->component_id, &rsp,
                                           0, 0, 0, 0, 0, flight_custom_version, 
                                           middleware_custom_version, os_custom_version, 0, 0, 
                                           uid, uid2);
        MsgAck ret(ack, rsp);
        return ret;
      } else if (cmd.command == MAV_CMD_REQUEST_FLIGHT_INFORMATION) { // flight information
        mavlink_message_t ack;
        mavlink_msg_command_ack_pack(this->system_id, this->component_id, &ack, cmd.command, 
                                     0, 0, 0, 0, 0);
        // TODO:: get flight_uuid ,the flight_uuid is related to the camera model  
        uint64_t flight_uuid = 1;
        auto now = std::chrono::system_clock::now();
        uint32_t time_boot_ms  = std::chrono::duration_cast<std::chrono::milliseconds> \
                                 (now.time_since_epoch()).count();
        mavlink_message_t rsp;
        mavlink_msg_flight_information_pack(this->system_id, this->component_id, &rsp, 
                                            time_boot_ms, 0, 0, flight_uuid);
        MsgAck ret(ack, rsp);
        return ret;
      } else if (cmd.command == MAV_CMD_SET_MESSAGE_INTERVAL) { // set msg interval
        uint16_t message_id = (uint16_t)cmd.param1; // MAVLINK_MSG_ID_ ...
        float interval_us   = cmd.param2;
        // TODO:: set message broadcast interval
        std::cout << "Set message broadcast interval msg_id: " << message_id <<
                     " interval_us: " << interval_us << "\n";

        mavlink_message_t ack;
        mavlink_msg_command_ack_pack(this->system_id, this->component_id, &ack, cmd.command,
                                     0, 0, 0, 0, 0);
        MsgAck ret(ack);
        return ret;
      }
    } // if comp id and sys id valid
    std::cout << "Received Command: ";
    std::cout << "[CMD ID] " << (int)cmd.command << " ";
    std::cout << "[VALUE] " << (int)cmd.param1 << "\n";
  } else{
    std::cout << "Message ID " << (int)msg.msgid << " received\n";
  }
  MsgAck ret;
  return ret;
}

} // namespace navatics_server
