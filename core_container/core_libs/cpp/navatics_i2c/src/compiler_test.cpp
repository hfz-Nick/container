#include "navatics_i2c/I2Cdev.h"
#include "navatics_i2c/devices/MS5837.h"
#include <iostream>

int main(){
  std::cout << "program running successfully\n";
  MS5837 dev();
  return 0;
}

