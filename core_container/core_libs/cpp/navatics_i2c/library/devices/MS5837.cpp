#include "navatics_i2c/devices/MS5837.h"

MS5837::MS5837(){
  this->default_d1_params = get_osr_params(MS5837_OSR_256);
  this->default_d2_params = get_osr_params(MS5837_OSR_256);
}

MS5837::MS5837(const char* device_name){
  this->i2cdev.initialize(device_name);
  this->reset();
  this->get_calibration_parameters();
  // set default params
  this->default_d1_params = get_osr_params(MS5837_OSR_256);
  this->default_d2_params = get_osr_params(MS5837_OSR_256);
} // MS5837 Constructor

void MS5837::initialize(const char* device_name){
  this->i2cdev.initialize(device_name);
  this->reset();
  this->get_calibration_parameters();
  // set default params
  this->default_d1_params = get_osr_params(MS5837_OSR_256);
  this->default_d2_params = get_osr_params(MS5837_OSR_256);
}

void MS5837::set_reading_resolution(uint8_t MS5837_TEMP_OSR, uint8_t MS5837_PRES_OSR){
  this->default_d1_params = get_osr_params(MS5837_PRES_OSR);
  this->default_d2_params = get_osr_params(MS5837_TEMP_OSR);
}

void MS5837::start_sensor_read_thread(){
  this->create_thread();
}

bool MS5837::reset(){
  uint8_t d = 0;
  return (this->i2cdev.writeByte(MS5837_ADDRESS_DEFAULT, MS5837_RESET_CMD, d));
} // bool reset()

void MS5837::get_calibration_parameters(){
  int length = 2;
  uint8_t data[length];
  // from C1 to C6 in order
  i2cdev.readBytes(MS5837_ADDRESS_DEFAULT, MS5837_C1_READ, length, data);
  this->C1 = (data[0] << 8 | data[1]);
  i2cdev.readBytes(MS5837_ADDRESS_DEFAULT, MS5837_C2_READ, length, data);
  this->C2 = (data[0] << 8 | data[1]);
  i2cdev.readBytes(MS5837_ADDRESS_DEFAULT, MS5837_C3_READ, length, data);
  this->C3 = (data[0] << 8 | data[1]);
  i2cdev.readBytes(MS5837_ADDRESS_DEFAULT, MS5837_C4_READ, length, data);
  this->C4 = (data[0] << 8 | data[1]);
  i2cdev.readBytes(MS5837_ADDRESS_DEFAULT, MS5837_C5_READ, length, data);
  this->C5 = (data[0] << 8 | data[1]);
  i2cdev.readBytes(MS5837_ADDRESS_DEFAULT, MS5837_C6_READ, length, data);
  this->C6 = (data[0] << 8 | data[1]);
} // void get_calibration_parameters()

MS5837OSRParameters MS5837::get_osr_params(uint8_t MS5837_OSR){
  MS5837OSRParameters osr_params;
  switch(MS5837_OSR){
    case (MS5837_OSR_256):
      osr_params.delay_us = MS5837_OSR_256_DELAY;
      osr_params.D1_osr_register = MS5837_D1_OSR_256;
      osr_params.D2_osr_register = MS5837_D2_OSR_256; 
      break;
    case (MS5837_OSR_512):
      osr_params.delay_us = MS5837_OSR_512_DELAY;
      osr_params.D1_osr_register = MS5837_D1_OSR_512;
      osr_params.D2_osr_register = MS5837_D2_OSR_512; 
      break;
    case (MS5837_OSR_1024):
      osr_params.delay_us = MS5837_OSR_1024_DELAY;
      osr_params.D1_osr_register = MS5837_D1_OSR_1024;
      osr_params.D2_osr_register = MS5837_D2_OSR_1024; 
      break;
    case (MS5837_OSR_2048):
      osr_params.delay_us = MS5837_OSR_2048_DELAY;
      osr_params.D1_osr_register = MS5837_D1_OSR_2048;
      osr_params.D2_osr_register = MS5837_D2_OSR_2048; 
      break;
    case (MS5837_OSR_4096):
      osr_params.delay_us = MS5837_OSR_4096_DELAY;
      osr_params.D1_osr_register = MS5837_D1_OSR_4096;
      osr_params.D2_osr_register = MS5837_D2_OSR_4096; 
      break;
    case (MS5837_OSR_8192):
      osr_params.delay_us = MS5837_OSR_8192_DELAY;
      osr_params.D1_osr_register = MS5837_D1_OSR_8192;
      osr_params.D2_osr_register = MS5837_D2_OSR_8192; 
      break;
    default:
      break;
  } // switch(MS5837_OSR)
  return (osr_params);
}

void MS5837::get_d1_adc_reading(uint8_t MS5837_OSR){
  int length = 3;
  uint8_t data[length];
  MS5837OSRParameters osr_params = get_osr_params(MS5837_OSR);
  i2cdev.writeByte(MS5837_ADDRESS_DEFAULT, osr_params.D1_osr_register, 0);
  std::this_thread::sleep_for(std::chrono::microseconds(osr_params.delay_us));
  i2cdev.readBytes(MS5837_ADDRESS_DEFAULT, MS5837_ADC_READ, length, data);
  this->D1 = (data[0] << 16 | data[1] << 8 | data[2]);
} // void get_d1_adc_reading()

void MS5837::get_d2_adc_reading(uint8_t MS5837_OSR){
  int length = 3;
  uint8_t data[length];
  MS5837OSRParameters osr_params = get_osr_params(MS5837_OSR);
  i2cdev.writeByte(MS5837_ADDRESS_DEFAULT, osr_params.D2_osr_register, 0);
  std::this_thread::sleep_for(std::chrono::microseconds(osr_params.delay_us));
  i2cdev.readBytes(MS5837_ADDRESS_DEFAULT, MS5837_ADC_READ, length, data);
  int D2 = (data[0] << 16 | data[1] << 8 | data[2]);
  this-> dT = D2 - (this->C5 << 8);
} // void get_d2_adc_reading()

void MS5837::get_d1_adc_reading(){
  int length = 3;
  uint8_t data[length];
  i2cdev.writeByte(MS5837_ADDRESS_DEFAULT, this->default_d1_params.D1_osr_register, 0);
  std::this_thread::sleep_for(std::chrono::microseconds(this->default_d1_params.delay_us));
  i2cdev.readBytes(MS5837_ADDRESS_DEFAULT, MS5837_ADC_READ, length, data);
  this->D1 = (data[0] << 16 | data[1] << 8 | data[2]);
} // void get_d1_adc_reading()

void MS5837::get_d2_adc_reading(){
  int length = 3;
  uint8_t data[length];
  i2cdev.writeByte(MS5837_ADDRESS_DEFAULT, this->default_d2_params.D2_osr_register, 0);
  std::this_thread::sleep_for(std::chrono::microseconds(this->default_d2_params.delay_us));
  i2cdev.readBytes(MS5837_ADDRESS_DEFAULT, MS5837_ADC_READ, length, data);
  int D2 = (data[0] << 16 | data[1] << 8 | data[2]);
  this-> dT = D2 - (this->C5 << 8);
} // void get_d1_adc_reading()

float MS5837::get_temperature(){
  return float((2000 + this->dT*float(this->C6)/(1<<23))/100);
} // float get_temperature()

float MS5837::get_pressure(){
  int OFF = (this->C2<<16) + this->dT*float(this->C4)/(1<<7);
  int SENS = (this->C1<<15) + this->dT*float(this->C3)/(1<<8);
  int P = (this->D1 * float(SENS)/(1<<21) - OFF)/(1<<13);
  return (float(P)/10000);
} // float get_pressure()

void *MS5837::get_adc_reading(void *t){
  while(1){
    ((MS5837 *) t)->get_d1_adc_reading();
    ((MS5837 *) t)->get_d2_adc_reading();
  }
} // get_adc_reading()

void MS5837::create_thread(){
  pthread_t thread_id;
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
  int rc = pthread_create(&thread_id, &attr, &MS5837::get_adc_reading, this);
} // void create adc read thread
