#include "navatics_robot_controller/thrusters.hpp"

namespace navatics_robot_controller{

Thrusters::Thrusters(rclcpp::Node * node){
  this->declare_params(node);
  this->get_params(node);

  // initialize serial device
  this->serial_device.setPort(this->serial_port);
  this->serial_device.setBaudrate( (uint32_t)this->serial_baudrate );
  serial::Timeout timeout = serial::Timeout::simpleTimeout(this->serial_timeout_ms);
  this->serial_device.setTimeout(timeout);
  this->serial_device.open();
  this->buffer_len = this->rpm_msg_len*2 + 5;

  // switch subscriber
  using namespace std::placeholders;
  this->rpm_switch_sub = node->create_subscription<std_msgs::msg::Bool>
                         ( "controller/rpm/switch", 50,
                           std::bind(&Thrusters::rpm_switch_sub_callback, this, _1) );
  // initialized publishers
  // std::string topic_name = "can/transmit";
  // this->can_bus_publisher = node->create_publisher<navatics_msgs::msg::CanBus>(topic_name, 1000);
  std::string status_topic = "controller/rpm/active";
  this->active_status_publisher = node->create_publisher<std_msgs::msg::Bool>(status_topic, 1000);
  this->last_status_time = this->get_system_time();
} // constructor

void Thrusters::rpm_switch_sub_callback(const std_msgs::msg::Bool::SharedPtr msg){
  this->is_thrusters_active = msg->data;
} // rpm_switch_sub_callback

void Thrusters::publish_rpm(Eigen::VectorXi rpm){
  if (rpm.rows() == this->n_thrusters){
    // calculate and properly assign rpm_msg according to msg_assignment
    uint8_t rpm_msg[this->rpm_msg_len*2];
    for (int j=0; j<this->rpm_msg_len; j++){
      int rpm_ = rpm(this->msg_assignment[j]);
      rpm_msg[2*j] = rpm(this->msg_assignment[j]) >> 8 & 0xFF;
      rpm_msg[2*j + 1] = rpm(this->msg_assignment[j]) & 0xFF;
    }
    uint8_t buffer[this->buffer_len];
    int i = 0;
    uint8_t msg_len = this->buffer_len - 5; // 4 start/stop bytes, 1 len
    buffer[i] = USART_STARTBYTES_0;
    i ++;
    buffer[i] = USART_STARTBYTES_1;
    i++;
    buffer[i] = (uint8_t)msg_len;
    i++;
    // fill in the buffer with rpm_msg
    for (int j=0; j < this->rpm_msg_len; j++){
      buffer[i] = rpm_msg[2*j];
      i++;
      buffer[i] = rpm_msg[2*j+1];
      i++;
    }
    buffer[i] = USART_STOPBYTES_0;
    i ++;
    buffer[i] = USART_STOPBYTES_1;
    i ++;
    if (i == this->buffer_len){
      this->serial_device.write(buffer, i);
    }
    
  } // if rpm length is appropriate

  // publish status message (active/inactive)
  float dt = this->get_time_diff(this->get_system_time(), this->last_status_time);
  if (dt > 1/this->status_update_hz){
    auto msg = std_msgs::msg::Bool();
    msg.data = this->is_thrusters_active;
    this->active_status_publisher->publish(msg);
    this->last_status_time = this->get_system_time();
  }
} // publish_rpm

Eigen::VectorXi Thrusters::calculate_rpm(Eigen::Vector3f force, Eigen::Vector3f moment){
  Eigen::VectorXf thrust = this->calculate_thrust(force, moment);
  Eigen::VectorXi rpm(this->n_thrusters);
  if (this->is_thrusters_active){
    for (int i=0; i<this->n_thrusters; i++)
      rpm(i) = this->rpm_lookup(thrust[i])*this->rpm_mod[i];
  } else {
    rpm = Eigen::VectorXi::Zero(this->n_thrusters);
  }
  return rpm;
} // calculate rpm

int Thrusters::rpm_lookup(float force){
  if (force > this->thrust_lut[0])
    return rpm_lut[0];
  else if (force < this->thrust_lut[this->n_lut-1])
    return rpm_lut[this->n_lut-1];
  int i=0;
  for (i=1; i<this->n_lut; i++){
    if (force > this->thrust_lut[i])
      break;
  }
  float df = this->thrust_lut[i-1] - this->thrust_lut[i];
  float dr = this->rpm_lut[i-1] - this->rpm_lut[i];
  float delta_f = force - this->thrust_lut[i];   
  return int(this->rpm_lut[i] + delta_f*dr/df);
} // rpm_lookup

Eigen::VectorXf Thrusters::calculate_thrust(Eigen::Vector3f force, Eigen::Vector3f moment){
  // vector assignment
  Eigen::VectorXf T(6);
  Eigen::Vector3f sat_force = this->saturate(force, this->max_force, this->min_force);
  Eigen::Vector3f sat_moment = this->saturate(moment, this->max_moment, this->min_moment);
  for (int i=0; i<3; i++){
    T(i) = sat_force[i];
    T(3+i) = sat_moment[i];
  } // assign force and moment to a 6x1 vetorA
  // initialize a vector with size (n_thrusters, 1)
  Eigen::VectorXf thrust(this->n_thrusters);
  thrust = this->force_tf_matrix * T;
  return thrust;
} // calculate thrust

Eigen::Vector3f Thrusters::saturate(Eigen::Vector3f input, Eigen::Vector3f max_saturation,
                                    Eigen::Vector3f min_saturation)
{
  Eigen::Vector3f result = input;
  for (int i=0; i<3; i++){
    if (input[i] > max_saturation[i])
      result[i] = max_saturation[i];
    else if (input[i] < min_saturation[i])
      result[i] = min_saturation[i];
  } // saturation iterator
  return result;
} // saturate

void Thrusters::declare_params(rclcpp::Node * node){
  node->declare_parameter("thrusters.transformation_matrix.column");
  node->declare_parameter("thrusters.transformation_matrix.row");
  node->declare_parameter("thrusters.transformation_matrix.data");
  node->declare_parameter("thrusters.transformation_matrix.id_list");
  node->declare_parameter("thrusters.transformation_matrix.rpm_mod");
  node->declare_parameter("thrusters.lookup_table.rpm");
  node->declare_parameter("thrusters.lookup_table.thrust");
  node->declare_parameter("thrusters.saturation.max_directional_force");
  node->declare_parameter("thrusters.saturation.min_directional_force");
  node->declare_parameter("thrusters.saturation.max_directional_moment");
  node->declare_parameter("thrusters.saturation.min_directional_moment");
  node->declare_parameter("thrusters.serial.port");
  node->declare_parameter("thrusters.serial.baudrate");
  node->declare_parameter("thrusters.serial.timeout_ms");
  node->declare_parameter("thrusters.serial.rpm_msg_assignment");
} // declare_params

void Thrusters::get_params(rclcpp::Node * node){
  int row, col;
  node->get_parameter("thrusters.transformation_matrix.column", col); 
  node->get_parameter("thrusters.transformation_matrix.row", row); 
  rclcpp::Parameter p("", std::vector <float>({0}));
  node->get_parameter("thrusters.transformation_matrix.data", p);
  // initialize transformation matrix based on parameters
  this->n_thrusters = row;
  this->force_tf_matrix = Eigen::MatrixXf(row, col);
  std::vector <double> data = p.as_double_array();
  for (int i=0; i<row; i++){
    for (int j=0; j<col; j++)
      this->force_tf_matrix(i,j) = data[i*col + j];
  } // initialize tf matrix
  // id and rpm modifier parameters
  rclcpp::Parameter pi("", std::vector <int>({0}));
  node->get_parameter("thrusters.transformation_matrix.id_list", pi);
  this->thrusters_id = std::vector<int>(pi.as_integer_array().begin(), pi.as_integer_array().end());
  this->unique_id = std::set<int>(thrusters_id.begin(), thrusters_id.end());
  // get indexes for each unique id in set
  for (std::set<int>::iterator it = this->unique_id.begin(); it!= this->unique_id.end(); it ++){
    std::vector <int> _indexes;
    for (int i=0; i<this->n_thrusters; i++){
      if (this->thrusters_id[i] == *it)
        _indexes.push_back(i);
    } // iterate over thrusters_id
    this->id_index_map.push_back(_indexes);
  } // iterate over unique set of id
  // get rpm modifier and place it into eigen placeholder
  node->get_parameter("thrusters.transformation_matrix.rpm_mod", pi);
  this->rpm_mod = std::vector<int>(pi.as_integer_array().begin(), pi.as_integer_array().end());
  // lookup table parameters
  node->get_parameter("thrusters.lookup_table.rpm", pi);
  this->rpm_lut = std::vector<int16_t>(pi.as_integer_array().begin(), pi.as_integer_array().end());
  node->get_parameter("thrusters.lookup_table.thrust", p);
  this->thrust_lut = p.as_double_array();
  this->n_lut = this->rpm_lut.size();
  if (n_lut != thrust_lut.size()){
    std::cout << "Error, lookup table does not return equal length, shutting down\n";
    rclcpp::shutdown();
  } // LUT size check
  // get saturation parameters
  node->get_parameter("thrusters.saturation.max_directional_moment", p);
  this->max_moment = Eigen::Vector3d(p.as_double_array().data()).cast <float> ();
  node->get_parameter("thrusters.saturation.min_directional_moment", p);
  this->min_moment = Eigen::Vector3d(p.as_double_array().data()).cast <float> ();
  node->get_parameter("thrusters.saturation.max_directional_force", p);
  this->max_force = Eigen::Vector3d(p.as_double_array().data()).cast <float> ();
  node->get_parameter("thrusters.saturation.min_directional_force", p);
  this->min_force = Eigen::Vector3d(p.as_double_array().data()).cast <float> ();
  // get serial parameters
  node->get_parameter_or("thrusters.serial.port", this->serial_port, std::string("/dev/ttyUSB0"));
  node->get_parameter_or("thrusters.serial.baudrate", this->serial_baudrate, 115200);
  node->get_parameter_or("thrusters.serial.timeout_ms", this->serial_timeout_ms, 1000);
  // get msg assignment params
  rclcpp::Parameter msg_a("", std::vector <int>({0}));
  node->get_parameter("thrusters.serial.rpm_msg_assignment", msg_a);
  this->msg_assignment = msg_a.as_integer_array();
  this->rpm_msg_len = this->msg_assignment.size();
  std::cout << "[PARAM] Msg Assignment Sequence: ";
  for (int i=0; i < msg_assignment.size(); i++){
    std::cout << msg_assignment[i] << " ";
  }
  std::cout << "\n";
} // get_params

std::chrono::time_point<std::chrono::system_clock> Thrusters::get_system_time(){
  return std::chrono::system_clock::now();
} // get_system_time

float Thrusters::get_time_diff(auto end_time, auto start_time){
  auto us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
  return (float(us/1e6));
} // get time diff


} // namespace navatics_robot_controller
