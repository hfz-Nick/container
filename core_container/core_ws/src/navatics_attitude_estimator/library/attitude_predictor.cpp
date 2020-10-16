#include "navatics_attitude_estimator/attitude_predictor.hpp"

namespace navatics_attitude_estimator{

AttitudePredictor::AttitudePredictor(rclcpp::Node * node, 
                                     uint8_t mpu_dev_addr, std::string mpu_dev_port,
                                     std::string sensor_name, std::string sensor_dir,
                                     bool is_reading_acc, uint8_t acc_range, float acc_update_hz,
                                     bool is_reading_gyro, uint8_t gyro_range, float gyro_update_hz,
                                     bool is_reading_mag, float mag_update_hz, bool is_debug,
                                     bool is_testing_rotation)
{
  // initialize i2c device
  this->mpu_dev_addr = mpu_dev_addr;
  this->mpu_dev_port = mpu_dev_port;
  this->mpu = MPU9150(mpu_dev_addr); 
  this->mpu.initialize(mpu_dev_port.c_str(), is_reading_mag);
  this->sensor_name = sensor_name;

  // define filename
  this->gyro_calib_filename = sensor_dir + "/" + this->sensor_name + "/calib/gyro.calib";
  this->mag_calib_filename = sensor_dir + "/" + this->sensor_name + "/calib/mag.calib";

  // set full scale range of sensors. Magnetometer only have 1 full scale range
  // set full scale range of accelerometer
  this->mpu.setFullScaleAccelRange(acc_range);
  switch (acc_range){
    case (ACC_RANGE_4G)  : this->acc_range =  4*G; break;
    case (ACC_RANGE_8G)  : this->acc_range =  8*G; break;
    case (ACC_RANGE_16G) : this->acc_range = 16*G; break;
    default: this->acc_range = 2*G; break;
  }
  // set full scale range of gyroscope
  this->mpu.setFullScaleGyroRange(gyro_range); 
  switch (gyro_range){
    case (GYRO_RANGE_500DPS)  : this->gyro_range =  500*DEG_TO_RAD; break;
    case (GYRO_RANGE_1000DPS) : this->gyro_range = 1000*DEG_TO_RAD; break;
    case (GYRO_RANGE_2000DPS) : this->gyro_range = 2000*DEG_TO_RAD; break;
    default: this->gyro_range = 250*DEG_TO_RAD; break;
  }

  // create timer instances
  // acc
  using namespace std::chrono_literals;
  using namespace std::placeholders;
  this->is_reading_acc = is_reading_acc;
  this->acc_update_hz = acc_update_hz;
  if (this->is_reading_acc){
    // initialize storage var
    this->acc_vector << 0, 0, 9.8;
    // create timer
    std::chrono::milliseconds duration_ms((int)(1000/this->acc_update_hz));
    this->acc_timer = node->create_wall_timer(
                        duration_ms,
                        std::bind(&AttitudePredictor::acc_read_callback, this)
                      );
    std::string topic_name = this->sensor_name + "/acc";
    this->acc_publisher = node->create_publisher<geometry_msgs::msg::Vector3>(topic_name, 100);
  } // if reading acc

  // gyro
  this->is_reading_gyro = is_reading_gyro;
  this->gyro_update_hz = gyro_update_hz;
  if (this->is_reading_gyro){
    // read calibration file
    std::cout << this->sensor_name << " loading gyroscope calibration\n";
    this->load_calibration_file(this->gyro_calib_filename, GYRO_SELECT);
    this->gyro_vector << 0,0,0;
    std::chrono::milliseconds duration_ms((int)(1000/this->gyro_update_hz));
    this->gyro_timer = node->create_wall_timer(
                         duration_ms,
                         std::bind(&AttitudePredictor::gyro_read_callback, this)
                      );
    std::string topic_name = this->sensor_name + "/gyro";
    this->gyro_publisher = node->create_publisher<geometry_msgs::msg::Vector3>(topic_name, 100);
  } // if reading gyro
  
  // mag
  this->is_reading_mag = is_reading_mag;
  this->mag_update_hz = mag_update_hz;
  if (this->is_reading_mag){
    // read calibration file
    std::cout << this->sensor_name << " loading magnetometer calibration\n";
    this->load_calibration_file(this->mag_calib_filename, MAG_SELECT);
    this->mag_vector << 0,0,0;
    std::chrono::milliseconds duration_ms((int)(1000/this->mag_update_hz));
    this->mag_timer = node->create_wall_timer(
                        duration_ms,
                        std::bind(&AttitudePredictor::mag_read_callback, this)
                      );
    std::string topic_name = this->sensor_name + "/mag";
    this->mag_publisher = node->create_publisher<geometry_msgs::msg::Vector3>(topic_name, 100);
  } // if reading mag

  this->is_debug = is_debug;
  if (this->is_debug)
    std::cout << "Started in Debug Mode\n";
} // AttitudePredictor Constructor

int AttitudePredictor::mpu_test(){
  // turn off debug to allow users to see the results
  this->is_debug = false;
  std::cout << "Testing MPU with ID " << int(this->mpu_dev_addr) << "\n";
  bool value_err = false;
  // check full scale accelerometer range
  std::cout << "Testing Full Scale Range of Sensors\n";
  uint8_t acc_range = this->mpu.getFullScaleAccelRange(); 
  uint8_t gyro_range = this->mpu.getFullScaleGyroRange(); 
  float max_acc_val, max_gyro_val;
  // set full scale range of accelerometer
  this->mpu.setFullScaleAccelRange(acc_range);
  switch (acc_range){
    case (ACC_RANGE_4G)  : max_acc_val =  4*G; break;
    case (ACC_RANGE_8G)  : max_acc_val =  8*G; break;
    case (ACC_RANGE_16G) : max_acc_val = 16*G; break;
    default: max_acc_val = 2*G; break;
  }
  if (fabs(max_acc_val - this->acc_range) < 0.1){
    std::cout << "Maximum acceleration value correct\n";
  } else {
    std::cout << "Maximum acceleration value incorrect\n";
    value_err = true;
  }
  // set full scale range of gyroscope
  this->mpu.setFullScaleGyroRange(gyro_range);
  switch (gyro_range){
    case (GYRO_RANGE_500DPS)  : max_gyro_val =  500*DEG_TO_RAD; break;
    case (GYRO_RANGE_1000DPS) : max_gyro_val = 1000*DEG_TO_RAD; break;
    case (GYRO_RANGE_2000DPS) : max_gyro_val = 2000*DEG_TO_RAD; break;
    default: max_gyro_val = 250*DEG_TO_RAD; break;
  }
  if (fabs(max_gyro_val - this->gyro_range) < 0.1){
    std::cout << "Maximum gyroscope value correct\n";
  } else {
    std::cout << "Maximum gyroscope value incorrect\n";
    value_err = true;
  }
  // test reading
  int16_t buffer[3];
  this->mpu.getAcceleration(&buffer[0], &buffer[1], &buffer[2]);
  Eigen::Vector3f acc_vec = this->update_reading(buffer, this->acc_range/MAX_BIT_VAL);
  if (acc_vec.norm() > 9 && acc_vec.norm() < 11){
    std::cout << "Accelerometer reading OK\n";
  } else {
    std::cout << "Accelerometer reading error\n";
    value_err = true;
  }
  this->mpu.getRotation(&buffer[0], &buffer[1], &buffer[2]);
  Eigen::Vector3f gyro_vec = this->update_reading(buffer, this->gyro_range/MAX_BIT_VAL);
  if (gyro_vec.norm() < 0.5){
    std::cout << "Gyroscope reading OK\n";
  } else {
    std::cout << "Gyroscope reading error\n";
    value_err = true;
  }
  // exit test
  if (!value_err){
    std::cout << "MPU device " << int(this->mpu_dev_addr) << " OK\n\n";
    return 0;
  } else {
    std::cout << "MPU device " << int(this->mpu_dev_addr) << " test exited with value error\n\n";
    return 1;
  }
} // mpu_test

int AttitudePredictor::get_rotation_test(float acc_diff_threshold, float mag_diff_threshold,
                                         float gyro_diff_threshold){
  int acc_test_result = 0, mag_test_result = 0, gyro_test_result = 0;
  std::default_random_engine gen;
  std::uniform_int_distribution<int> dist(-100, 100);
  // Testing accelerometer
  Eigen::Quaternionf q_test(1.0f, (float)dist(gen)/100, (float)dist(gen)/100, (float)dist(gen)/100);
  q_test.normalize();
  Eigen::Matrix3f R = q_test.toRotationMatrix();
  Eigen::Vector3f north(R(0,0), R(0,1), R(0,2));
  Eigen::Vector3f down(R(2,0), R(2,1), R(2,2));
  Eigen::Quaternionf q_test_result = this->get_acc_rotation(north, down);
  if ( (fabs( q_test.w() - q_test_result.w() ) > acc_diff_threshold) or
       (fabs( q_test.x() - q_test_result.x() ) > acc_diff_threshold) or
       (fabs( q_test.y() - q_test_result.y() ) > acc_diff_threshold) or
       (fabs( q_test.z() - q_test_result.z() ) > acc_diff_threshold) ) { 
    std::cout << "Accelerometer rotation calculation test failed\n";
    acc_test_result = 1;
  } else {
    std::cout << "Accelerometer rotation calculation test succeeded\n";
    acc_test_result = 0;
  } // accelerometer condition check
  
  // Testing magnetometer
  q_test = Eigen::Quaternionf(1.0f, (float)dist(gen)/100, (float)dist(gen)/100, (float)dist(gen)/100);
  q_test.normalize();
  R = q_test.toRotationMatrix();
  north << R(0,0), R(0,1), R(0,2);
  down << R(2,0), R(2,1), R(2,2);
  q_test_result = this->get_mag_rotation(north, down);
  if ( (fabs( q_test.w() - q_test_result.w() ) > mag_diff_threshold) or
       (fabs( q_test.x() - q_test_result.x() ) > mag_diff_threshold) or
       (fabs( q_test.y() - q_test_result.y() ) > mag_diff_threshold) or
       (fabs( q_test.z() - q_test_result.z() ) > mag_diff_threshold) )
  { 
    std::cout << "Magnetometer rotation calculation test failed\n";
    mag_test_result = 1;
  } else {
    std::cout << "Magnetometer rotation calculation test succeeded\n";
    mag_test_result = 0;
  } // magnetometer condition check
  
  // Testing Gyroscope
  Eigen::Vector3f w((float)dist(gen)/100, (float)dist(gen)/100, 0.4);
  float dt = (float)dist(gen)/100;
  Eigen::AngleAxisf _aa(w.norm()*dt, w.normalized());
  q_test = Eigen::Quaternionf(_aa);
  q_test_result = this->get_gyro_rotation(w, Eigen::Quaternionf(1,0,0,0), dt);
  if ( (fabs( q_test.w() - q_test_result.w() ) > gyro_diff_threshold) or
       (fabs( q_test.x() - q_test_result.x() ) > gyro_diff_threshold) or
       (fabs( q_test.y() - q_test_result.y() ) > gyro_diff_threshold) or
       (fabs( q_test.z() - q_test_result.z() ) > gyro_diff_threshold) )
  { 
    std::cout << "Gyroscope rotation calculation test failed\n";
    gyro_test_result = 1;
  } else {
    std::cout << "Gyroscope rotation calculation test succeeded\n";
    gyro_test_result = 0;
  } // gyroscope condition check
  
  return acc_test_result + gyro_test_result*2 + mag_test_result*4;
} // state_update_test

void AttitudePredictor::debug(bool is_debug){
  this->is_debug = is_debug;
} // debug

void AttitudePredictor::initialize(Orientation *state){
  this->link_state(state);
  // get sensor reading
  this->acc_vector = this->get_sensor_reading(ACC_SELECT);
  this->gyro_vector = this->get_sensor_reading(GYRO_SELECT);
  // calculate rotation
  Eigen::Vector3f down = this->acc_vector.normalized();
  Eigen::Vector3f north;
  Eigen::Quaternionf q;
  if (this->is_reading_mag and this->is_mag_calibrated){
    this->mag_vector = this->get_sensor_reading(MAG_SELECT);
    north = this->mag_vector.normalized();
    q = get_mag_rotation(north, down);
  } else {
    north << 1, 0, 0;
    q = get_acc_rotation(north, down);
  }
  std::cout << "Orientation initialized at: ";
  std::cout << q.w() << " " << q.x() << " " << q.y() << " " << q.z()  << "\n";
  this->state->initialize(q, this->gyro_vector);
} // initialize

void AttitudePredictor::link_state(Orientation *state){
  // link state to orientation object
  this->state = state;
}

void AttitudePredictor::set_update_weights(float gyro, float gyro_angular_velocity, 
                                           float acc, float mag)
{
  this->w_gyro = gyro;
  this->w_angular_vel_gyro = gyro_angular_velocity;
  this->w_acc = acc;
  this->w_mag = mag;
} // set_update_weights

Eigen::Vector3f AttitudePredictor::get_sensor_reading(int sensor_type){
  int16_t buffer[3];
  Eigen::Vector3f result(0,0,0);
  Eigen::Vector3f vec;
  switch(sensor_type){
    case(ACC_SELECT):
      this->mpu.getAcceleration(&buffer[0], &buffer[1], &buffer[2]);
      vec = this->update_reading(buffer, this->acc_range/MAX_BIT_VAL);
      result << -vec[0], vec[1], vec[2];
      break;
    case(GYRO_SELECT):
      this->mpu.getRotation(&buffer[0], &buffer[1], &buffer[2]);
      vec = this->update_reading(buffer, this->gyro_range/MAX_BIT_VAL);
      result << vec[0], -vec[1], -vec[2];
      if (this->is_gyro_calibrated)
        result -= this->gyro_bias;
      break;
    case(MAG_SELECT):
      this->mpu.getMagnetometer(&buffer[0], &buffer[1], &buffer[2]);
      vec = this->update_reading(buffer, MAG_RANGE/MAX_MAG_BIT_VAL);
      result << vec[1], -vec[0], vec[2];
      if (this->is_mag_calibrated)
        result -= this->mag_bias;
      break;
  } // switch type
  return result;
} // get_sensor_reading

void AttitudePredictor::acc_read_callback(){
  this->acc_vector = this->get_sensor_reading(ACC_SELECT);
  if (this->state->initialized()){
    Eigen::Quaternionf q = this->state->get_orientation();
    Eigen::Matrix3f R = q.toRotationMatrix();
    Eigen::Vector3f north(R(0,0), R(0,1), R(0,2));
    Eigen::Vector3f down = this->acc_vector.normalized();
    q = this->get_acc_rotation(north, down);
    this->state->update_orientation(q, this->w_acc);
  }
  if (this->is_debug){
    std::cout << int(this->mpu_dev_addr) << " ";
    std::cout << "acc: ";
    std::cout << this->acc_vector.transpose() << "\n";
  }
} // acc_read_callback

void AttitudePredictor::gyro_read_callback(){
  // calculate time difference
  auto current_time = get_system_time();
  float dt = get_time_diff(current_time, this->start_time);
  if (dt > 1)
    dt = 0.01; // dummy value for the first data
  this->start_time = current_time;
  // read data
  this->gyro_vector = this->get_sensor_reading(GYRO_SELECT);
  if (this->state->initialized()){
    this->state->update_orientation(this->get_gyro_rotation(this->gyro_vector, 
                                                            this->state->get_orientation(), dt), 
                                    this->w_gyro);
    this->state->update_angular_velocity(this->gyro_vector, this->w_angular_vel_gyro);
  }
  // debug
  if (this->is_debug){
    std::cout << int(this->mpu_dev_addr) << " ";
    std::cout << "gyro: ";
    std::cout << this->gyro_vector.transpose() << "\n";
  }
  // calibration
  if (this->is_calibrating_gyro){
    this->gyro_vec_container.push_back(this->gyro_vector);
    if (this->gyro_vec_container.size() == 200){
      this->update_gyro_bias();
    }
  } // if calibrating gyroscope
} // gyro_read_callback

void AttitudePredictor::mag_read_callback(){
  this->mag_vector = this->get_sensor_reading(MAG_SELECT);
  if (this->is_debug){
    std::cout << int(this->mpu_dev_addr) << " ";
    std::cout << "mag: ";
    std::cout << this->mag_vector.transpose() << "\n";
  }
  if (this->state->initialized() and this->is_mag_calibrated){
    Eigen::Quaternionf q = this->state->get_orientation();
    Eigen::Matrix3f R = q.toRotationMatrix();
    Eigen::Vector3f down(R(2,0), R(2,1), R(2,2));
    if (this->is_reading_acc)
      down = this->acc_vector.normalized();
    Eigen::Vector3f north = this->mag_vector.normalized();
    this->state->update_orientation(this->get_mag_rotation(north, down), w_mag);
  }
  // calibration
  if (this->is_calibrating_mag) {
    float duration = this->get_time_diff(this->get_system_time(), this->mag_calib_start_time);
    if (duration > this->mag_calib_timeout_sec){
      std::cout << "calibration timeout, reloading previous calibration params (if any)\n";
      this->load_calibration_file(this->mag_calib_filename, MAG_SELECT);
      this->is_calibrating_mag = false;
      this->mx_container.clear(); this->my_container.clear(); this->mz_container.clear();
    }
    if (this->mx_container.size() == 0 && this->is_calibrating_mag){
      // blindly push back the first vector
      this->mx_container.push_back(this->mag_vector[0]);
      this->my_container.push_back(this->mag_vector[1]);
      this->mz_container.push_back(this->mag_vector[2]);
    } else {
      // if data pass the check, push back vector and calculate parameters
      if (this->get_mag_data(this->mag_vector)) {
        this->mx_container.push_back(this->mag_vector[0]);
        this->my_container.push_back(this->mag_vector[1]);
        this->mz_container.push_back(this->mag_vector[2]);
        // check if there's enough data for calculation
        if (this->mx_container.size() >= this->mag_min_exit_len){
          // calculate center of magnetometers
          float cx = ( *std::max_element(this->mx_container.begin(), this->mx_container.end()) +
                       *std::min_element(this->mx_container.begin(), this->mx_container.end()) ) / 2;
          float cy = ( *std::max_element(this->my_container.begin(), this->my_container.end()) +
                       *std::min_element(this->my_container.begin(), this->my_container.end()) ) / 2;
          float cz = ( *std::max_element(this->mz_container.begin(), this->mz_container.end()) +
                       *std::min_element(this->mz_container.begin(), this->mz_container.end()) ) / 2;
          std::vector <bool> flags (this->mag_theta_reference.size(), false);
          // iterate over all entry and get radius of each entry to the center
          // use only those with radius > threshold
          for (int i=0; i<this->mx_container.size() && this->is_mag_calibrated == false; i++){
            float radius = std::sqrt( std::pow((this->mx_container[i] - cx), 2)+
                                      std::pow((this->my_container[i] - cy), 2)+
                                      std::pow((this->mz_container[i] - cz), 2) );
            if (radius > this->mag_radius_threshold){
              float theta = atan2(this->my_container[i] - cy, this->mx_container[i] - cx);
              if (this->is_mag_calibrated = this->get_mag_exit_status(&flags, theta))
                this->is_calibrating_mag = false;
              if (this->is_mag_calibrated){
                std::cout << this->sensor_name << " magnetometer calibrated, bias: ";
                this->mag_bias << cx, cy, cz;
                std::cout << this->mag_bias.transpose() << "\n";
                this->write_calibration_file(this->mag_calib_filename, this->mag_bias, 
                                             this->is_mag_calibrated);
              }
            } // if radius > threshold
          } // iterate over entry

        } // calculation of center and radius of mag data
      } // if mag data valid
    } // if not first entry
  } // mag calibration

} // mag_read_callback

Eigen::Vector3f AttitudePredictor::update_reading(int16_t buffer[3], float scaler){
  Eigen::Vector3f vec;
  vec << buffer[0], buffer[1], buffer[2];
  return (vec*scaler);
} // update_reading

Eigen::Quaternionf AttitudePredictor::get_acc_rotation(Eigen::Vector3f north, Eigen::Vector3f down)
{
  // normalize inputs
  Eigen::Vector3f _north = north.normalized();
  Eigen::Vector3f _down = down.normalized();
  // get cross product
  Eigen::Vector3f _east = (_down.cross(_north)).normalized();
  // get and transpose rotation matrix, use to initialize quaternion
  Eigen::Matrix3f R;
  R << _north, _east, _down;
  R.transposeInPlace();
  Eigen::Quaternionf q(R);
  return q;
} // get_acc_rotation

Eigen::Quaternionf AttitudePredictor::get_mag_rotation(Eigen::Vector3f north, Eigen::Vector3f down)
{
  // normalize inputs
  Eigen::Vector3f _north = north.normalized();
  Eigen::Vector3f _down = down.normalized();
  // project north down to plane perpendicular to m_down vector
  float g = _down.dot(_north);
  Eigen::Vector3f projected_north = _north - g*_down;
  projected_north.normalize();
  // get east, construct rotation matrix
  Eigen::Vector3f _east = _down.cross(projected_north);
  _east.normalize();
  // restore north
  _north = _east.cross(_down);
  _north.normalize();
  // create rotation matrix and convert to quaternion
  Eigen::Matrix3f R;
  R << _north, _east, _down;
  R.transposeInPlace();
  Eigen::Quaternionf q(R);
  return q;
}

Eigen::Quaternionf AttitudePredictor::get_gyro_rotation(Eigen::Vector3f w, Eigen::Quaternionf q,
                                                        float dt)
{
  Eigen::Vector3f r_axis = w;
  float r_angle = r_axis.norm()*dt;
  r_axis.normalize();
  Eigen::AngleAxisf _aa(r_angle, r_axis);
  Eigen::Quaternionf _qe(_aa);
  _qe.normalize();
  Eigen::Quaternionf gyro_rotation = q*_qe;
  return(gyro_rotation.normalized());
} // get_gyro_rotation

bool AttitudePredictor::is_mpu_calibrated(){
  bool _is_gyro_calibrated, _is_mag_calibrated;
  if (not this->is_reading_gyro){
    _is_gyro_calibrated = true;
  } else {
    _is_gyro_calibrated = this->is_gyro_calibrated;
  } // gyro
  
  if (not this->is_reading_mag){
    _is_mag_calibrated = true;
  } else {
    _is_mag_calibrated = this->is_mag_calibrated;
  } // mag

  return (_is_gyro_calibrated and _is_mag_calibrated);
} // bool is_mpu_calibrated

std::chrono::time_point<std::chrono::system_clock> AttitudePredictor::get_system_time(){
  return std::chrono::system_clock::now();
} // get_system_time

float AttitudePredictor::get_time_diff(auto end_time, auto start_time){
  auto us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
  return (float(us/1e6));
} // get time diff

void AttitudePredictor::calibrate_gyroscope(){
  if (!this->is_calibrating_gyro and this->is_reading_gyro){
    std::cout << this->sensor_name << " Calibrating gyroscope\n";
    this->is_calibrating_gyro = true;
    this->is_gyro_calibrated = false;
    this->gyro_vec_container.clear();
  }
} // calibrate_gyroscope

void AttitudePredictor::calibrate_magnetometer(){
  if (!this->is_calibrating_mag and this->is_reading_mag){
    std::cout << this->sensor_name << " Calibrating magnetometer\n";
    this->is_calibrating_mag = true;
    this->is_mag_calibrated = false;
    this->mag_calib_start_time = this->get_system_time();
    this->mx_container.clear(); this->my_container.clear(); this->mz_container.clear();
  }
} // calibrate_magnetometer

bool AttitudePredictor::update_gyro_bias(){
  // toggle calibrating flag
  this->is_calibrating_gyro = false;
  // calibration logic
  std::vector<Eigen::Vector3f>::iterator it;
  Eigen::Vector3f sum (0,0,0);
  for (it = this->gyro_vec_container.begin(); it < this->gyro_vec_container.end(); it++){
    sum += *it;
  }
  Eigen::Vector3f bias = sum*(1.0f/this->gyro_vec_container.size());
  Eigen::Vector3f stdev (0,0,0);
  for (it = this->gyro_vec_container.begin(); it < this->gyro_vec_container.end(); it++){
    Eigen::Vector3f tmp = (*it - bias).array().square();
    stdev += tmp;
  }
  stdev = stdev * (1.0f/this->gyro_vec_container.size());
  stdev = stdev.array().sqrt();
  std::cout << stdev.transpose() << "\n";
  // clear gyro vector container
  this->gyro_vec_container.clear();
  // bias validation condition
  bool is_valid = false;
  if (stdev.norm() < this->gyro_bias_threshold){
    this->is_gyro_calibrated = true;
    this->gyro_bias = bias;
    // write to file
    this->write_calibration_file(this->gyro_calib_filename, bias);
    is_valid = true;
  }
  // debug
  if (is_valid){
    std::cout << this->sensor_name << " Gyroscope calibrated ";
    std::cout << "bias: " << bias.transpose() << "\n";
  } else {
    std::cout << this->sensor_name << " Gyroscope calibration failed\n";
  }
  return is_valid;
} // update_gyro_bias

bool AttitudePredictor::get_mag_data(Eigen::Vector3f vector){
  if (fabs(vector[0]) > 500 || fabs(vector[1]) > 500 || fabs(vector[2]) >500)
    return false;
  for (int i=0; i<this->mx_container.size(); i++){
    float dx = vector[0]-this->mx_container[i];
    float dy = vector[1]-this->my_container[i];
    float dz = vector[2]-this->mz_container[i];     
    if (sqrt(dx*dx + dy*dy + dz*dz) < this->mag_distance_threshold)       
      return false;   
  }
  return true;
} // get_mag_data

bool AttitudePredictor::get_mag_exit_status(std::vector <bool> * flags, float theta){
  bool is_searching = true;
  for (int i=0; i<this->mag_theta_reference.size() && is_searching; i++){
    // if flag is false and the angle is in range
    if( !((*flags)[i]) && 
        (fabs(theta - this->mag_theta_reference[i]) < this->mag_theta_ref_threshold) )
    {       
      (*flags)[i] = true;
      is_searching = false;
    }
  } // iterate over mag theta reference and toggle flag to true if theta found
  for (int i=0; i<this->mag_theta_reference.size(); i++){
    if(!((*flags)[i]))       
      return false;   
  } // check if all theta is found
  return true;
}

void AttitudePredictor::load_calibration_file(std::string filename, int sensor_type){
  std::ifstream calib_file(filename);
  std::string line;
  std::vector <std::string> line_container;
  Eigen::Vector3f bias;
  bool flag;
  bool is_read_successful = false;
  if (calib_file.is_open()){
    while(std::getline(calib_file, line) and line_container.size() < 4)
      line_container.push_back(line);
    calib_file.close();
    if (line_container.size() > 0 and line_container[0] == "true"){
      flag = true;
      for (int i=0; i<3; i++)
        bias[i] = std::stof(line_container[i+1]);
      is_read_successful = true;
    } else {
      std::cout << "load failed\n";
      flag = false;
      bias << 0,0,0;
    }
  } else {
    std::cout << "file cannot be opened\n";
    flag = false;
    bias << 0,0,0;
  } // file cannot be opened

  switch (sensor_type){
    case (GYRO_SELECT):
      this->is_gyro_calibrated = flag; this->gyro_bias = bias; break;
    case (MAG_SELECT):
      this->is_mag_calibrated = flag; this->mag_bias = bias; break;
  } // update sensor biases
  if (is_read_successful)
    std::cout << "load successful\n";
} // load_calibration_file

void AttitudePredictor::write_calibration_file(std::string filename, Eigen::Vector3f bias, bool flag){
  std::ofstream output_file;
  output_file.open(filename);
  std::string str_flag = "false";
  if (flag)
    str_flag = "true";
  output_file << str_flag << "\n" << bias[0] << "\n" << bias[1] << "\n" << bias[2];
  output_file.close();
} // write_calibration_file

void AttitudePredictor::publish_sensor_data(){
  auto message = geometry_msgs::msg::Vector3();
  if (this->is_reading_acc){
    message.x = this->acc_vector[0];
    message.y = this->acc_vector[1];
    message.z = this->acc_vector[2];
    this->acc_publisher->publish(message);
  } // acc publisher
  if (this->is_reading_gyro){
    message.x = this->gyro_vector[0];
    message.y = this->gyro_vector[1];
    message.z = this->gyro_vector[2];
    this->gyro_publisher->publish(message);
  } // gyro publisher
  if (this->is_reading_mag){
    message.x = this->mag_vector[0];
    message.y = this->mag_vector[1];
    message.z = this->mag_vector[2];
    this->mag_publisher->publish(message);
  } // mag publisher
} // publish_sensor_data

} // navatics_attitude_estimator
