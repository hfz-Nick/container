#ifndef NAVATICSSERVER_MAVLINKSERVER_H
#define NAVATICSSERVER_MAVLINKSERVER_H

#include <rclcpp/rclcpp.hpp>

#include "navatics_server/mavlink_msg_handler.hpp"
#include "navatics_server/mavlink_state_pub.hpp"

#include <string>
#include <cstring>
#include <thread>
#include <chrono>

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <time.h>
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#include <stdbool.h>

#include <mavlink/common/mavlink.h>

namespace navatics_server{

#define BUFFER_LENGTH 2041


class MAVLinkServer{

  public:
  MAVLinkServer(rclcpp::Node * node, std::string remote_ip, int remote_port, int local_port);

  // function used to update loop rate of specific published item, and its enumerated selection
  void set_loop_rate(int update_variable_selection, float rate_hz);
  enum rate_selection{
    STATE_UPDATE_HZ,
    REFSTATE_UPDATE_HZ
  };

  private:
  // mavlink variables
  char remote_ip[100] = {"255.255.255.255"};
  struct sockaddr_in local_addr;
  struct sockaddr_in remote_addr;
  int local_port = 15000;
  int remote_port = 15001;
  int sock = 0;
  void initialize_socket_addresses();

  // message variables
  uint8_t system_id = 0x01;
  uint8_t component_id = 0x01; // component ID 1 for autopilot
  uint8_t component_type = MAV_TYPE_SUBMARINE; // MAV type sumbarine
  uint8_t component_autopilot_type = MAV_AUTOPILOT_GENERIC; // MAV autopilot generic
  uint8_t component_control_mode = MAV_MODE_STABILIZE_ARMED;
  uint8_t component_state = MAV_STATE_STANDBY;

  // heartbeat timer
  rclcpp::TimerBase::SharedPtr heartbeat_loop;
  void heartbeat_loop_callback();
  bool is_connected = false;
  std::chrono::time_point<std::chrono::system_clock> received_heartbeat_time;
  float time_diff_threshold = 5.0; // second

  // boot time
  std::chrono::time_point<std::chrono::system_clock> bootup_time;
  
  // time
  std::chrono::time_point<std::chrono::system_clock> get_system_time();
  float get_time_diff(auto end_time, auto start_time);

  // receiver thread
  static void * udp_receiver_thread(void *t);
  void create_receiver_thread();
  std::vector <mavlink_message_t> msg_queue;
  MAVLinkMsgHandler *handler;

  // sender function
  void send_udp_packet(mavlink_message_t msg);

  // state publishers
  MAVLinkStatePub *state_pub;
  rclcpp::TimerBase::SharedPtr update_loop;
  void update_loop_callback();
  int update_counter = 0;
  float state_update_hz = 20;
  float refstate_update_hz = 20;

 
}; // class MAVLinkServer

} // namespace navatics_server

#endif // NAVATICSSERVER_MAVLINKSERVER_H
