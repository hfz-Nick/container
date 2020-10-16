#include "navatics_can_interface/can_interface_manager.hpp"
#include "navatics_can_interface/can_message.hpp"

namespace navatics_can_interface{

CANInterfaceManager::CANInterfaceManager(rclcpp::Node * node) {
  // set serial device port and baudrate
  this->declare_params(node);
  this->get_params(node);
  // set up serial device
  this->serial_device.setPort(this->serial_port);
  this->serial_device.setBaudrate( (uint32_t)this->serial_baudrate );
  serial::Timeout timeout = serial::Timeout::simpleTimeout(this->serial_timeout_ms);
  this->serial_device.setTimeout(timeout);
  this->serial_device.open();
  // ROS subscriber and timer
  using namespace std::placeholders;
  // 
  std::chrono::milliseconds transmit_duration_ms((int)(1000/this->transmitter_update_hz));
  std::chrono::milliseconds receive_duration_ms((int)(1000/this->receiver_update_hz));
  // subscriber and publisher
  this->can_interface_sub = node->create_subscription<navatics_msgs::msg::CanBus>
                            ("can/transmit", 1000,
                             std::bind(&CANInterfaceManager::can_interface_sub_callback, this, _1));
  this->can_receiver_pub = node->create_publisher<navatics_msgs::msg::CanBus>("can/received", 1000);
  this->create_thread();  
  // loops used to publish received messange and write to serial subscribed topic
  // these loops are currently disabled due to memory leak issues
  // this->can_transmitter_loop = node->create_wall_timer( transmit_duration_ms,
  //                              std::bind(&CANInterfaceManager::can_transmitter_loop_callback, this));
  // this->can_receiver_loop = node->create_wall_timer( receive_duration_ms,
  //                             std::bind(&CANInterfaceManager::can_receiver_loop_callback, this) );
} // constructor

void CANInterfaceManager::can_receiver_loop_callback(){
  int s = this->receiver_queue.size();
  if (s != 0 and s < 50){
    this->publish_can_message(this->receiver_queue.back());
    this->receiver_queue.erase(this->receiver_queue.end());
  } else if (s > 50){
    this->receiver_queue.clear();
  }
} // can_receiver_loop_callback

void CANInterfaceManager::can_transmitter_loop_callback(){
  // this loop handles the sending of message received from ROS to CAN Bus via USART
  if (!this->transmission_queue.empty()){
    CANMessage msg = this->transmission_queue[0];
    this->transmission_queue.erase(this->transmission_queue.begin());
    this->write_serial(msg);
  }
} // can_transmitter_loop_callback

void CANInterfaceManager::write_serial(CANMessage msg){
  // constructing USART message
  // adding length of redundancies like start and stop bytes, length, and another 2 bytes for id
  int n = msg.len + this->redundancy_length + 2;
  uint8_t buffer[n];
  int i = 0;
  buffer[i] = USART_STARTBYTES_0;
  i ++;
  buffer[i] = USART_STARTBYTES_1;
  i ++;
  buffer[i] = msg.len + 2; // add 2 length for ID
  i ++;
  buffer[i] = (msg.std_id >> 8 & 0xFF);
  i ++;
  buffer[i] = (msg.std_id & 0xFF);
  i++;
  for (int j=0; j < msg.len; j++){
    buffer[i] = msg.data[j];
    i++;
  }
  buffer[i] = USART_STOPBYTES_0;
  i++;
  buffer[i] = USART_STOPBYTES_1;
  // send buffer as a USART message
  this->serial_device.write(buffer, n);
} // write_serial

void CANInterfaceManager::can_interface_sub_callback(const navatics_msgs::msg::CanBus::SharedPtr msg){
  // this callback handles putting message received from ROS in transmission queue
  std::vector <uint8_t> data(msg->data);
  uint8_t length = msg->length;
  uint16_t id = msg->std_id;
  CANMessage message(id, length, data);
  if (message.valid()){
    // this->transmission_queue.push_back(message);
    this->write_serial(message);
  } else {
    std::cout << "invalid message\n";
  }
} // can_interface_sub_callback

void CANInterfaceManager::parse_sys_message(CANMessage msg){
  if (msg.std_id == 0x0000){
    std::cout << "Shutting Down Computer\n";
    std::fstream fs;
    fs.open("/tmp/shutdown_cmd.txt");
    fs << 1;
    fs.close();
  }
} // parse_sys_message

void CANInterfaceManager::publish_can_message(CANMessage message){
  if (message.valid()){
    auto msg = navatics_msgs::msg::CanBus();
    msg.std_id = message.std_id;
    msg.length = message.len;
    msg.data = message.data;
    this->can_receiver_pub->publish(msg);
  } else {
    std::cout << "message received invalid \n";
  }
} // publish_can_message

void *CANInterfaceManager::get_serial_reading(void *t){
  while(1){
    uint8_t buffer;
    size_t result;
    result = ((CANInterfaceManager *) t)->serial_device.read(&buffer, 1);
    // message received
    if (result == 1){
      // receive logic

      // start message
      if ( buffer == USART_STARTBYTES_1 &&
           ((CANInterfaceManager *) t)->previous_message == USART_STARTBYTES_0  &&
           ((CANInterfaceManager *) t)->is_start_message_received == false  )
      {
        // start message flag
        ((CANInterfaceManager *) t)->is_start_message_received = true;
      } // start message

      // message length
      else if ( ((CANInterfaceManager *) t)->is_start_message_received == true &&
                ((CANInterfaceManager *) t)->is_message_length_received == false )
      {
        // check message length
        if (buffer > 1 && buffer < 11){ // buffer between 2 and 10
          ((CANInterfaceManager *) t)->is_message_length_received = true;
          ((CANInterfaceManager *) t)->msg_len = buffer - 2;
        }
        else{
          ((CANInterfaceManager *) t)->reset_usart_handler_flag();
        }
      } // message length


      // message id
      else if ( ((CANInterfaceManager *) t)->is_message_length_received == true &&
                ((CANInterfaceManager *) t)->is_message_id_high_received == false )
      {
        // receive high message id
        ((CANInterfaceManager *) t)->is_message_id_high_received = true;
        ((CANInterfaceManager *) t)->msg_id = buffer << 8;
      } // message id high

      else if ( ((CANInterfaceManager *) t)->is_message_id_high_received == true &&
                ((CANInterfaceManager *) t)->is_message_id_low_received == false )
      {
        // receive low message id
        ((CANInterfaceManager *) t)->is_message_id_low_received = true;
        ((CANInterfaceManager *) t)->msg_id = ( ((CANInterfaceManager *) t)->msg_id | buffer );
        // check id fidelity
        if ( ((CANInterfaceManager *) t)->msg_id > 0x07FF )
          ((CANInterfaceManager *) t)->reset_usart_handler_flag();
      } // message id low

      // data
      else if ( ((CANInterfaceManager *) t)->is_message_id_low_received == true &&
                ((CANInterfaceManager *) t)->is_message_id_low_received == true &&
                ((CANInterfaceManager *) t)->msg_data.size() <
                ((CANInterfaceManager *) t)->msg_len )
      {
        ((CANInterfaceManager *) t)->msg_data.push_back(buffer);
        if ( ((CANInterfaceManager *) t)->msg_data.size() == ((CANInterfaceManager *) t)->msg_len ){
          ((CANInterfaceManager *) t)->is_message_data_received = true;
        }
      } // data

      // stop message
      else if ( ((CANInterfaceManager *) t)->is_message_data_received = true &&
                buffer == USART_STOPBYTES_1 &&
                ((CANInterfaceManager *) t)->previous_message == USART_STOPBYTES_0 )
      {
        // generate ROS CAN message
        CANMessage msg;
        msg.std_id = ((CANInterfaceManager *) t)->msg_id;
        msg.len = ((CANInterfaceManager *) t)->msg_len;
        msg.data = ((CANInterfaceManager *) t)->msg_data;
	if (msg.std_id < 0x00FF){
	  // parse system message
          ((CANInterfaceManager *) t)->parse_sys_message(msg);
	} else {
          ((CANInterfaceManager *) t)->publish_can_message(msg);
	}
          // ((CANInterfaceManager *) t)->receiver_queue.push_back(msg);

        // clear flags
        ((CANInterfaceManager *) t)->reset_usart_handler_flag();
      } // stop message

      // keep message as previous message
      ((CANInterfaceManager *) t)->previous_message = buffer;
    } // if message received
  } // while loop
} // get serial reading thread


void CANInterfaceManager::reset_usart_handler_flag(){
  this->is_start_message_received = false;
  this->is_message_length_received = false;
  this->is_message_id_low_received = false;
  this->is_message_id_high_received = false;
  this->is_message_data_received = false;
  this->msg_len = 0;
  this->msg_id = 0;
  this->msg_data.clear();
} // void reset_usart_handler_flag

void CANInterfaceManager::create_thread(){
  pthread_t thread_id;
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
  int rc = pthread_create(&thread_id, &attr, &CANInterfaceManager::get_serial_reading, this);
} // void create read thread

void CANInterfaceManager::declare_params(rclcpp::Node * node){
  node->declare_parameter("serial.port");
  node->declare_parameter("serial.baudrate");
  node->declare_parameter("serial.timeout_ms");
  node->declare_parameter("transmitter.update_hz");
  node->declare_parameter("receiver.update_hz");
} // declare_params

void CANInterfaceManager::get_params(rclcpp::Node * node){
  node->get_parameter_or("serial.port", this->serial_port, std::string("/dev/ttyUSB0"));
  std::cout << "[PARAM] CAN Interface Port: " << this->serial_port << "\n";
  node->get_parameter_or("serial.baudrate", this->serial_baudrate, 115200);
  std::cout << "[PARAM] CAN Interface Baudrate: " << this->serial_baudrate << "\n";
  node->get_parameter_or("serial.timeout_ms", this->serial_timeout_ms, 1000);
  node->get_parameter_or("transmitter.update_hz", this->transmitter_update_hz, 1000.0f);
  node->get_parameter_or("receiver.update_hz", this->receiver_update_hz, 1000.0f);
} // get_params

} // namespace navatics_can_interface
