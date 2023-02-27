#ifndef LSM9DS1_DRIVER_H
#define LSM9DS1_DRIVER_H

#include "registers.hpp"

#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

namespace LSM9DS1Namespace {

struct SensorData {
  int accelX;
  int accelY;
  int accelZ;
  int gyroX;
  int gyroY;
  int gyroZ;
};

class LSM9DS1 {
public:
  LSM9DS1(const std::string &spiDev);
  SensorData readSensorData();

private:
  int spiFd;
  uint8_t spiMode;
  uint8_t spiBits;
  uint32_t spiSpeed;
  uint8_t spiCs;

  void spiTransfer(uint8_t* txBuf, uint8_t* rxBuf, int len);
};

} // namespace LSM9DS1Namespace

#endif  // LSM9DS1_DRIVER_H