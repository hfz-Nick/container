#ifndef NAVATICSCANINTERFACE_CANINTERFACEMANAGER_H
#define NAVATICSCANINTERFACE_CANINTERFACEMANAGER_H

#include <rclcpp/rclcpp.hpp>

#include <serial/serial.h>


#include <string>
#include <vector>
#include <fstream>

#include "navatics_can_interface/can_message.hpp"
#include "navatics_msgs/msg/can_bus.hpp"

namespace navatics_can_interface{

#define USART_STARTBYTES_0 0xBE
#define USART_STARTBYTES_1 0xEF
#define USART_STOPBYTES_0 0xCA
#define USART_STOPBYTES_1 0xFE

class CANInterfaceManager{

  public:
  CANInterfaceManager(rclcpp::Node * node);

  private:
  // declare and get parameters
  void declare_params(rclcpp::Node * node);
  void get_params(rclcpp::Node * node);
  
  // serial device
  serial::Serial serial_device;
  std::string serial_port = "/dev/ttyUSB0";
  int serial_baudrate = 115200;
  int serial_timeout_ms = 1000;
  // ROS subscriber
  rclcpp::Subscription<navatics_msgs::msg::CanBus>::SharedPtr can_interface_sub;
  void can_interface_sub_callback(const navatics_msgs::msg::CanBus::SharedPtr msg);
  // ROS transmitter timer
  rclcpp::TimerBase::SharedPtr can_transmitter_loop;
  void can_transmitter_loop_callback();
  void transmit_can_message();
  float transmitter_update_hz = 1000.0;
  void write_serial(CANMessage msg);
  // ROS receiver timer and publisher
  rclcpp::TimerBase::SharedPtr can_receiver_loop;
  void can_receiver_loop_callback();
  rclcpp::Publisher<navatics_msgs::msg::CanBus>::SharedPtr can_receiver_pub;
  float receiver_update_hz = 1000.0;
  // for usart transmission usages
  int redundancy_length = 5; // 2 for startbytes, 2 for stopbytes, 1 for length
  // transmit queue
  std::vector <CANMessage> transmission_queue;
  // received message buffer
  std::vector <CANMessage> receiver_queue;
  void publish_can_message(CANMessage msg);
  void parse_sys_message(CANMessage msg);

  // thread for receive
  static void * get_serial_reading(void *t);
  void create_thread();
  // received USART message handler
  uint8_t previous_message = 0x00;
  bool is_start_message_received = false;
  bool is_message_length_received = false;
  int msg_len;
  bool is_message_id_high_received = false;
  bool is_message_id_low_received = false;
  uint16_t msg_id;
  bool is_message_data_received = false;
  std::vector <uint8_t> msg_data;
  void reset_usart_handler_flag();

}; // class CANInterfaceManager

} // namespace navatics_can_interface

#endif // NAVATICSCANINTERFACE_CANINTERFACEMANAGER_H

