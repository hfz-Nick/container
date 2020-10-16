#ifndef NAVATICSI2C_DEVICES_MS5837_H
#define NAVATICSI2C_DEVICES_MS5837_H

#include "navatics_i2c/I2Cdev.h"
#include <string>
#include <thread>
#include <chrono>

#define MS5837_ADDRESS_LOW        0x76
#define MS5837_ADDRESS_HIGH       0x77
#define MS5837_ADDRESS_DEFAULT    MS5837_ADDRESS_LOW

// Define factory calibration register address
#define MS5837_CRC_READ   0xA0
#define MS5837_C1_READ    0xA2
#define MS5837_C2_READ    0xA4
#define MS5837_C3_READ    0xA6
#define MS5837_C4_READ    0xA8
#define MS5837_C5_READ    0xAA
#define MS5837_C6_READ    0xAC

// Define Commands Register
#define MS5837_RESET_CMD    0x1E
#define MS5837_D1_OSR_256   0x40
#define MS5837_D1_OSR_512   0x42
#define MS5837_D1_OSR_1024  0x44
#define MS5837_D1_OSR_2048  0x46
#define MS5837_D1_OSR_4096  0x48
#define MS5837_D1_OSR_8192  0x4A
#define MS5837_D2_OSR_256   0x50
#define MS5837_D2_OSR_512   0x52
#define MS5837_D2_OSR_1024  0x54
#define MS5837_D2_OSR_2048  0x56
#define MS5837_D2_OSR_4096  0x58
#define MS5837_D2_OSR_8192  0x5A
#define MS5837_ADC_READ     0x00

// Define Delay depending on OSR (in microsecs)
#define MS5837_OSR_256_DELAY     600
#define MS5837_OSR_512_DELAY    1170
#define MS5837_OSR_1024_DELAY   2280
#define MS5837_OSR_2048_DELAY   4540
#define MS5837_OSR_4096_DELAY   9040
#define MS5837_OSR_8192_DELAY  18080

// Define MS5837 OSR
#define MS5837_OSR_256    0x00
#define MS5837_OSR_512    0x01
#define MS5837_OSR_1024   0x02
#define MS5837_OSR_2048   0x03
#define MS5837_OSR_4096   0x04
#define MS5837_OSR_8192   0x05     

struct MS5837OSRParameters{
  int delay_us;
  uint8_t D1_osr_register;
  uint8_t D2_osr_register;
};

class MS5837 {
  public:
    MS5837();
    MS5837(const char* device_name);
    void initialize(const char* device_name = "/dev/i2c-0");
    void set_reading_resolution(uint8_t MS5837_TEMP_OSR, uint8_t MS5837_PRES_OSR);
    void start_sensor_read_thread();
    // I2C commands
    bool reset();
    void get_calibration_parameters();
    void get_d1_adc_reading(uint8_t MS5837_OSR);
    void get_d1_adc_reading();
    void get_d2_adc_reading(uint8_t MS5837_OSR);
    void get_d2_adc_reading();
    // conversion commands
    float get_pressure();
    float get_temperature();

  private:
    // thread functions
    void create_thread();
    static void * get_adc_reading(void *t);
    // i2c device
    navatics_i2c::I2Cdev i2cdev;
    // OSR parameter selection
    MS5837OSRParameters get_osr_params(uint8_t MS5837_OSR);
    MS5837OSRParameters default_d1_params;
    MS5837OSRParameters default_d2_params;
    // calibration parameters
    unsigned int C1, C2, C3, C4, C5, C6;
    // reading parameters
    int dT, D1;
  
};

#endif // NAVATICSI2C_DEVICES_MS5837_H
