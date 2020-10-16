#include "navatics_position_estimator/depth_predictor.hpp"

namespace navatics_position_estimator{

DepthPredictor::DepthPredictor(rclcpp::Node * node, std::string sensor_name, 
                               uint8_t dev_addr, std::string dev_port, float pressure_update_hz){
  this->dev_addr = dev_addr;
  this->dev_port = dev_port;
  // initialize pressure sensor interface object
  this->psu.initialize(this->dev_port.c_str());
  this->psu.start_sensor_read_thread();
  // initialize depth read timer
  std::chrono::milliseconds duration_ms((int)(1000/this->pressure_update_hz));
  this->depth_timer = node->create_wall_timer(
                        duration_ms, std::bind(&DepthPredictor::depth_read_callback, this)
                      );
  // initialize depth and velocity storage variables
  this->depth = this->pressure_to_depth(this->psu.get_pressure());
  this->prev_depth = this->depth;
  this->velocity = 0.0f;

  // initialize publisher
  this->sensor_name = sensor_name;
  std::string topic_name = this->sensor_name + "/pressure";
  this->pressure_publisher = node->create_publisher<std_msgs::msg::Float64>(topic_name, 100);

  // check if temporary file exists
  this->calib_filename = "/tmp/" + this->sensor_name + "/calib.txt";
  std::fstream fs;
  fs.open(this->calib_filename);
  if (fs.is_open()){
    std::string s;
    std::getline(fs, s);
    try{
      this->pressure_bias = std::stof(s);
      this->is_calibrated = true;
      this->is_calibration_finished = true;
      std::cout << "calibrated from file " << this->calib_filename << "\n";
      std::cout << "pressure bias: " << this->pressure_bias << "\n";
    } catch(...){
      this->is_calibrated = false;
    }
  } else {
    this->is_calibrated = false;
  } // check if file is open
  this->calibration_array.clear();
  
  // initialize start time
  this->start_time = this->get_system_time();
} // constructor

int DepthPredictor::psu_test(){
  std::cout << "Testing PSU\n";
  float psu_reading = 0;
  usleep(1000000);
  int n_readings = 10;
  int error_reading = 0;
  for (int i=0; i<n_readings; i++){
    float p = this->psu.get_pressure();
    if (fabs(p) < 1.5)
      psu_reading += p;
    else
      error_reading += 1;
    usleep(100000);
  } // for loop, get 10 readings
  psu_reading /= n_readings;
  float error_pct = (float)error_reading/n_readings;
  std::cout << "mean: " << psu_reading << "\n";
  int ret_val = 0;
  if (fabs(psu_reading) > 1.5)
    ret_val += 1;
  if (error_pct > 0.1)
    ret_val += 2;
  return ret_val;
}

void DepthPredictor::initialize(Position *state){
  // First time the function is called, it's going to link the state to depth predictor
  if (!this->is_state_linked){
    this->link_state(state);
    this->is_state_linked = true;
  } // link state
  // if the calibration is finished, initialize directly,
  // otherwise toggle the init_state_after_calibration flag to init state after calibrate
  if (this->is_calibration_finished) {
    std::cout << "initialized state without calibrating (reading calib data from file)\n";
    this->initialize_linked_state();
  } else {
    std::cout << "waiting for calibration to finish\n";
    this->init_state_after_calibration = true;
  } // wait for calibration
} // initialize

void DepthPredictor::initialize_linked_state(){
  Eigen::Vector3f ff_position;
  ff_position << 0,0,this->depth;
  Eigen::Vector3f ff_velocity;
  ff_velocity << 0,0,this->velocity;
  this->state->initialize(ff_position, ff_velocity);
}

void DepthPredictor::link_state(Position *state){
  this->state = state;
} // link_state

void DepthPredictor::set_update_weights(float pos_gain, float vel_gain){
  this->pos_weight = pos_gain;
  this->vel_weight = vel_gain;
} // set_update_weight

void DepthPredictor::depth_read_callback(){
  // update depth and velocity reading
  this->pressure = psu.get_pressure();
  if (!this->is_calibrated){
    this->calibration_array.push_back(this->pressure);
    if (this->calibration_array.size() > this->n_calib_array_len){
      this->calibrate_pressure_sensor();
      this->is_calibration_finished = true;
      if (this->init_state_after_calibration){
        std::cout << "state initialized after calibration\n";
        this->initialize_linked_state();
      } // if not initialized, but state is linked
    } // if calibration is due
  } else {
    this->depth = this->mean_filter( this->pressure_to_depth(this->pressure - this->pressure_bias), 
                                     &this->depth_container, this->max_depth_container_size );
    auto t = this->get_system_time();
    float dt = this->get_time_diff(t, this->start_time);
    float new_velocity = (this->depth - this->prev_depth)/dt;
    this->velocity = this->mean_filter(new_velocity, &this->velocity_container, 
                                       this->max_velocity_container_size);
    this->prev_depth = this->depth;
    if (this->state->initialized()){
      Eigen::Vector3f ff_position, ff_velocity;
      ff_position << 0,0,this->depth;
      ff_velocity << 0,0,this->velocity;
      this->state->update_position(ff_position, this->pos_weight);
      this->state->update_velocity(ff_velocity, this->vel_weight);
    } // if state is initialized
  } // if calibrated
} // depth_read_callback

float DepthPredictor::pressure_to_depth(float pressure){
  return (pressure - P_ATM)*BAR_TO_METER;
} // pressure_to_depth

float DepthPredictor::mean_filter(float val, std::vector<float> * container, int max_container_size){
  float result = 0.0f;
  container->push_back(val);
  // pop first element if full
  if(container->size() > max_container_size)
    container->erase(container->begin());
  // calculate average
  std::vector<float>::iterator it;
  for (it = container->begin();  it < container->end(); it++)
    result += *it;
  return (result / container->size());
} // mean_filter

void DepthPredictor::publish_sensor_data(){
  auto msg = std_msgs::msg::Float64();
  msg.data = this->pressure - this->pressure_bias;
  this->pressure_publisher->publish(msg);
}

std::chrono::time_point<std::chrono::system_clock> DepthPredictor::get_system_time(){
  return std::chrono::system_clock::now();
} // get_system_time

float DepthPredictor::get_time_diff(auto end_time, auto start_time){
  auto us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
  return (float(us/1e6));
} // get time diff

void DepthPredictor::calibrate_pressure_sensor(){
  std::sort(this->calibration_array.begin(), this->calibration_array.end());
  int n = this->calibration_array.size();
  float median = this->calibration_array[(int)(n/2)];
  float sum_pressure_bias = 0;
  int count = 0;
  for (int i=0; i<n; i++){
    if (fabs(this->calibration_array[i] - median) < this->median_distance_threshold){
      sum_pressure_bias += this->calibration_array[i];
      count ++;
    } // if distance to median is lower than threshold
  } // iterate over array
  this->pressure_bias = sum_pressure_bias/count - P_ATM;
  if (this->pressure_bias < -0.7 or this->pressure_bias > 0.02)
    this->pressure_bias = 0; // not initialized if already underwater or error
  std::fstream fs;
  fs.open(this->calib_filename, std::ofstream::out | std::ofstream::trunc);
  fs << this->pressure_bias;
  fs.close();
  std::cout << "writing calibration data to file\n";
  this->is_calibrated = true;
  this->is_calibration_finished = true;
} // calibrate_pressure_sensor


} // namespace navatics_position_estimator
