#include "navatics_server/mavlink_server.hpp"

namespace navatics_server{

MAVLinkServer::MAVLinkServer(rclcpp::Node * node, std::string remote_ip, int remote_port, 
                             int local_port)
{
  std::strcpy(this->remote_ip, remote_ip.c_str());
  this->remote_port = remote_port;
  this->local_port = local_port;

  this->handler = new MAVLinkMsgHandler(node, "cmd_vel", "controller/rpm/switch", 
                                        this->system_id, this->component_id);
  this->state_pub = new MAVLinkStatePub(node);
  
  this->initialize_socket_addresses();
  this->create_receiver_thread();

  std::chrono::milliseconds heartbeat_duration(1000);
  this->heartbeat_loop = node->create_wall_timer( heartbeat_duration, 
                                                  std::bind(&MAVLinkServer::heartbeat_loop_callback, 
                                                            this) );

  // loop at 100 Hz
  std::chrono::milliseconds update_duration(10);
  this->update_loop = node->create_wall_timer( update_duration, 
                                               std::bind(&MAVLinkServer::update_loop_callback, 
                                                         this) );

  this->bootup_time = this->get_system_time();
  
} // constructor

void MAVLinkServer::set_loop_rate(int update_variable_selection, float rate_hz){
  switch (update_variable_selection){
    case(STATE_UPDATE_HZ): this->state_update_hz = rate_hz; break;
    case(REFSTATE_UPDATE_HZ): this->refstate_update_hz = rate_hz; break;
  } // switch-case update_variable_selection
}

void MAVLinkServer::update_loop_callback(){
  this->update_counter ++;
  mavlink_message_t msg;
  
  if (this->update_counter % (int)(100/this->state_update_hz) == 0){
    auto current_time = this->get_system_time();
    uint32_t bootup_duration = (uint32_t)this->get_time_diff(current_time, this->bootup_time);
    State s = this->state_pub->get_current_state();
    float q[4] = {s.qw, s.qx, s.qy, s.qz};
    float covariance[21] = {0};
    mavlink_msg_odometry_pack(this->system_id, this->component_id, &msg,
                               bootup_duration, 0, 0, s.nedx, s.nedy, s.nedz, q, 
                               s.vx, s.vy, s.vz, s.wx, s.wy, s.wz, covariance, covariance, 0, 0);
    this->send_udp_packet(msg);

    mavlink_msg_extended_sys_state_pack(this->system_id, this->component_id, &msg, 
                                        0, MAV_LANDED_STATE_ON_GROUND);
    this->send_udp_packet(msg);


  } // update counter for state

  if (this->update_counter % (int)(100/this->refstate_update_hz) == 0){
    auto current_time = this->get_system_time();
    uint32_t bootup_duration = (uint32_t)this->get_time_diff(current_time, this->bootup_time);
    State s = this->state_pub->get_current_refstate();
    float q[4] = {s.qw, s.qx, s.qy, s.qz};
    mavlink_msg_attitude_target_pack(this->system_id, this->component_id, &msg, bootup_duration, 0,
                                     q, s.wx, s.wy, s.wz, 0);
    this->send_udp_packet(msg);
    mavlink_msg_position_target_local_ned_pack(this->system_id, this->component_id, &msg, 
                                               bootup_duration, 1, 0, s.nedx, s.nedy, s.nedz, 
                                               s.vx, s.vy, s.vz, 0, 0, 0, 0, 0);
    this->send_udp_packet(msg);

    //TODO:: get gps info
    uint8_t fix_type = 4;
    uint8_t satellites_visible = 9;
    mavlink_msg_gps_raw_int_pack(this->system_id, this->component_id, &msg,
                                 bootup_duration, fix_type, 0, 0, 0, 0, 0, 0, 0, satellites_visible, 
                                 0, 0, 0, 0, 0, 0);
    this->send_udp_packet(msg);

    //TODO:: get home position info
    q[0] = 0; q[1] = 0; q[2] = 0; q[3] = 0;
    mavlink_msg_home_position_pack(this->system_id, this->component_id, &msg,
                               0, 0, 0, 0, 0, 0, q, 0, 0, 0, 0);
    this->send_udp_packet(msg);

    // update connectivity at the same rate as refstate_update_hz
    if (this->is_connected){
      // publish connected message
      this->handler->publish_connected();
    }
  } // update counter for refstate

  // after 500 ticks (~5s) update_counter= 0
  if (this->update_counter > 500)
    this->update_counter = 0; 
  
} // update_loop_callback

void MAVLinkServer::heartbeat_loop_callback(){
  mavlink_message_t msg;
  if (this->state_pub->is_mpu_calibrated)
    this->component_state = MAV_STATE_CALIBRATING;
  else
    this->component_state = MAV_STATE_STANDBY;

  uint8_t base_mode = this->component_control_mode;
  if (this->state_pub->is_thruster_active)
    base_mode = MAV_MODE_FLAG_SAFETY_ARMED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
  else
    base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
  if (base_mode != this->component_control_mode)
  {
    this->component_control_mode = base_mode;
    std::cout << "[INFO] CONTROL MODE CHANGE" << (int)base_mode << "\n";
  }
  uint32_t flightMode_manual = 0x10000;
  mavlink_msg_heartbeat_pack(this->system_id, this->component_id, &msg, this->component_type,
                             this->component_autopilot_type, this->component_control_mode, 
                             flightMode_manual, this->component_state);
  this->send_udp_packet(msg);
  
  // check if disconnected
  if (this->is_connected){
    auto current_time = this->get_system_time();
    float time_diff = this->get_time_diff(current_time, this->received_heartbeat_time);
    if (time_diff > this->time_diff_threshold){
      this->is_connected = false;
      this->handler->publish_disconnected();
      std::cout << "[ERROR] Ground Control System Disconnected\n";
    }
  }
} // heartbeat_loop_callback

void MAVLinkServer::send_udp_packet(mavlink_message_t msg){
  uint8_t buf[BUFFER_LENGTH];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  int bytes_sent = sendto(this->sock, buf, len, 0, (struct sockaddr*)&this->remote_addr,
                          sizeof(struct sockaddr_in));
} // send_udp_packet

void MAVLinkServer::initialize_socket_addresses(){

  memset(&this->local_addr, 0, sizeof(this->local_addr));
  this->local_addr.sin_family = AF_INET;
  this->local_addr.sin_addr.s_addr = INADDR_ANY;
  this->local_addr.sin_port = htons(this->local_port);

  memset(&this->remote_addr, 0, sizeof(this->remote_addr));
  this->remote_addr.sin_family = AF_INET;
  this->remote_addr.sin_addr.s_addr = inet_addr(this->remote_ip);
  this->remote_addr.sin_port = htons(this->remote_port);

  this->sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

  /* Try non-blocking proceses */
  if (fcntl(this->sock, F_SETFL, O_NONBLOCK | O_ASYNC) < 0){
    printf("[ERROR] setting nonblocking failed\n");
    close(this->sock);
    exit(EXIT_FAILURE);
  }

} // initialize_socket_addresses

void MAVLinkServer::create_receiver_thread(){
  pthread_t thread_id;
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
  int rc = pthread_create(&thread_id, &attr, &MAVLinkServer::udp_receiver_thread ,this);
} // create_receiver_thread

void *MAVLinkServer::udp_receiver_thread(void *t){
  struct sockaddr_in _remote_addr = ((MAVLinkServer *)t)->remote_addr;
  struct sockaddr_in _local_addr = ((MAVLinkServer *)t)->local_addr;
  int _sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

  int broadcast = 1;
  if(setsockopt(((MAVLinkServer *)t)->sock, SOL_SOCKET, SO_BROADCAST, 
                &broadcast, sizeof(broadcast)) < 0) {
    printf("[ERROR] Set Broadcast option failed\n");
    close( ((MAVLinkServer *)t)->sock );
    exit(EXIT_FAILURE);
  }


  if (-1 == bind(((MAVLinkServer *)t)->sock,(struct sockaddr *)&_local_addr, sizeof(struct sockaddr))) {
    printf("[ERROR] bind failed\n");
    close(_sock);
    exit(EXIT_FAILURE);
  }

  mavlink_message_t msg;
  uint16_t len;
  uint8_t buf[BUFFER_LENGTH];
  ssize_t recsize;
  socklen_t fromlen = sizeof(_remote_addr);
  unsigned int temp = 0;
  int bytes_sent;

  while(1){
    memset(buf, 0, BUFFER_LENGTH);
    recsize = recvfrom(((MAVLinkServer *)t)->sock, (void *)buf, BUFFER_LENGTH, 0,
                       (struct sockaddr *)&_remote_addr,&fromlen);
    if (recsize > 0) {
      // Something received - print out all bytes and parse packet
      mavlink_message_t msg;
      mavlink_status_t status;

      for (int i = 0; i < recsize; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
          if (msg.msgid == 0){
            if ( !((MAVLinkServer *)t)->is_connected ){
              ((MAVLinkServer *)t)->is_connected = true;
              std::cout << "Connected to Ground Control Station\n";
            }
            ((MAVLinkServer *)t)->received_heartbeat_time = ((MAVLinkServer *)t)->get_system_time();
          } // Heartbeat
          else{
            MsgAck msg_ack = ((MAVLinkServer *)t)->handler->parse(msg);
            if (msg_ack.enable_ack){
              ((MAVLinkServer *)t)->send_udp_packet(msg_ack.ack);
            }
            if (msg_ack.enable_msg) {
              ((MAVLinkServer *)t)->send_udp_packet(msg_ack.msg);
            }
          }
      }
    } // if received message
    memset(buf, 0, BUFFER_LENGTH);
  } // thread loop

} // udp_receiver_thread

/* Helper Functions */

std::chrono::time_point<std::chrono::system_clock> MAVLinkServer::get_system_time(){
  return std::chrono::system_clock::now();
} // get_system_time

float MAVLinkServer::get_time_diff(auto end_time, auto start_time){
  auto us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
  return (float(us/1e6));
} // get time diff

} // namespace navatics_server
