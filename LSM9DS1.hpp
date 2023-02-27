#ifndef LSM9DS1_DRIVER_H
#define LSM9DS1_DRIVER_H

#include "registers.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

namespace LSM9DS1Namespace {

struct SensorData {
  int16_t accelX;
  int16_t accelY;
  int16_t accelZ;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;
};

struct SensorDataScaled {
  double accelX;
  double accelY;
  double accelZ;
  double gyroX;
  double gyroY;
  double gyroZ;
};

class LSM9DS1 {
public:
  LSM9DS1(const std::string &spiDev);
  ~LSM9DS1();
  bool verify();
  SensorData readSensorData();
  SensorDataScaled readSensorDataScaled();

private:
  void reset();
  int spiFd;
  uint8_t spiMode;
  uint8_t spiBits;
  uint32_t spiSpeed;
  uint8_t spiCs;

  void spiTransfer(uint8_t* txBuf, uint8_t* rxBuf, int len);
};

} // namespace LSM9DS1Namespace

#endif  // LSM9DS1_DRIVER_H