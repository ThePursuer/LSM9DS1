#include "LSM9DS1.hpp"

namespace LSM9DS1Namespace {

LSM9DS1::LSM9DS1(const std::string &spiDev) {
  // Open the SPI device file
  spiFd = open(spiDev.c_str(), O_RDWR);
  if (spiFd < 0) {
    std::cerr << "Failed to open SPI device" << std::endl;
    return;
  }

  // Set SPI mode, bits per word, and clock speed
  spiMode = SPI_MODE_3;
  spiBits = 8;
  spiSpeed = 1000000;
  if (ioctl(spiFd, SPI_IOC_WR_MODE, &spiMode) < 0) {
    std::cerr << "Failed to set SPI mode" << std::endl;
    close(spiFd);
    spiFd = -1;
    return;
  }
  if (ioctl(spiFd, SPI_IOC_WR_BITS_PER_WORD, &spiBits) < 0) {
    std::cerr << "Failed to set SPI bits per word" << std::endl;
    close(spiFd);
    spiFd = -1;
    return;
  }
  if (ioctl(spiFd, SPI_IOC_WR_MAX_SPEED_HZ, &spiSpeed) < 0) {
    std::cerr << "Failed to set SPI clock speed" << std::endl;
    close(spiFd);
    spiFd = -1;
    return;
  }
  reset();

  // Initialize the sensor
  uint8_t txBuf[2] = {0};
  uint8_t rxBuf[2] = {0};

  // Read WHO_AM_I register to verify sensor is connected
  txBuf[0] = LSM9DS1_WHO_AM_I | 0x80; // Read bit set
  spiTransfer(txBuf, rxBuf, 2);
  if (rxBuf[1] != 0x68) {
    std::cerr << "Failed to detect LSM9DS1 sensor" << std::endl;
    close(spiFd);
    spiFd = -1;
    return;
  }

  // Configure accelerometer
  txBuf[0] = LSM9DS1_CTRL_REG5_XL;
  txBuf[1] = LSM9DS1_CTRL_REG5_XL_ODR_119Hz | LSM9DS1_CTRL_REG5_XL_FS_2g;
  spiTransfer(txBuf, rxBuf, 2);
  txBuf[0] = LSM9DS1_CTRL_REG6_XL;
  txBuf[1] = 0x80; // Enable X, Y, and Z axes
  spiTransfer(txBuf, rxBuf, 2);

  // Configure gyroscope
  txBuf[0] = LSM9DS1_CTRL_REG1_G;
  txBuf[1] = LSM9DS1_CTRL_REG1_G_ODR_238Hz | LSM9DS1_CTRL_REG4_FS_G_245dps;
  spiTransfer(txBuf, rxBuf, 2);
  txBuf[0] = LSM9DS1_CTRL_REG4;
  txBuf[1] = LSM9DS1_CTRL_REG4_FS_G_245dps;
  spiTransfer(txBuf, rxBuf, 2);

  // // Enable interrupts (optional)
  // txBuf[0] = LSM9DS1_INT1_CTRL;
  // txBuf[1] = 0x01; // Interrupt on new data available
  // spiTransfer(txBuf, rxBuf, 2);
  
  // Enable accelerometer and gyroscope
  txBuf[0] = LSM9DS1_CTRL_REG1_G;
  txBuf[1] |= 0x0F; // Enable all axes
  spiTransfer(txBuf, rxBuf, 2);
  txBuf[0] = LSM9DS1_CTRL_REG5_XL;
  txBuf[1] |= 0x38; // Enable all axes
  spiTransfer(txBuf, rxBuf, 2);
}

void LSM9DS1::spiTransfer(uint8_t* txBuf, uint8_t* rxBuf, int len) {
  struct spi_ioc_transfer xfer = {
      .tx_buf = (unsigned long)txBuf,
      .rx_buf = (unsigned long)rxBuf,
      .len = len,
      .speed_hz = spiSpeed,
      .delay_usecs = 0,
      .bits_per_word = spiBits,
  };
  ioctl(spiFd, SPI_IOC_MESSAGE(1), &xfer);
}

SensorData LSM9DS1::readSensorData() {
  SensorData data;

  // Read accelerometer and gyro data
  uint8_t txBuf[13] = {0};
  uint8_t rxBuf[13] = {0};
  txBuf[0] = LSM9DS1_OUT_X_L_XL | 0x80 | 0x40; // Auto-increment address, read bit set
  spiTransfer(txBuf, rxBuf, 13);

  // Parse accelerometer data
  data.accelX = (int16_t)((rxBuf[2] << 8) | rxBuf[1]);
  data.accelY = (int16_t)((rxBuf[4] << 8) | rxBuf[3]);
  data.accelZ = (int16_t)((rxBuf[6] << 8) | rxBuf[5]);

  // Parse gyro data
  data.gyroX = (int16_t)((rxBuf[8] << 8) | rxBuf[7]);
  data.gyroY = (int16_t)((rxBuf[10] << 8) | rxBuf[9]);
  data.gyroZ = (int16_t)((rxBuf[12] << 8) | rxBuf[11]);

  return data;
}

SensorDataScaled LSM9DS1::readSensorDataScaled() {
  // Read raw sensor data
  auto data = readSensorData();

  // Scale accelerometer data
  double accelScale = 2.0 / 32768.0; // assuming +/-2g range
  SensorDataScaled scaledData;
  scaledData.accelX = static_cast<double>(data.accelX) * accelScale;
  scaledData.accelY = static_cast<double>(data.accelY) * accelScale;
  scaledData.accelZ = static_cast<double>(data.accelZ) * accelScale;

  // Scale gyro data
  double gyroScale = 245.0 / 32768.0; // assuming +/-245dps range
  scaledData.gyroX = static_cast<double>(data.gyroX) * gyroScale;
  scaledData.gyroY = static_cast<double>(data.gyroY) * gyroScale;
  scaledData.gyroZ = static_cast<double>(data.gyroZ) * gyroScale;

  // Return scaled sensor data
  return scaledData;
}

void LSM9DS1::reset() {
  // Write 0x05 to CTRL_REG8 to initiate software reset
  uint8_t txBuf[] = {LSM9DS1_CTRL_REG8, 0x05};
  uint8_t rxBuf[2];
  spiTransfer(txBuf, rxBuf, 2);

  // Wait for the reset to complete (100ms delay)
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

bool LSM9DS1::verify() {
  // Check accelerometer and gyroscope data ready status
  uint8_t txBuf[] = {LSM9DS1_OUT_X_L_XL};
  uint8_t rxBuf[7];
  spiTransfer(txBuf, rxBuf, 7);
  bool accelDataReady = (rxBuf[2] & (1 << 3)) != 0;
  bool gyroDataReady = (rxBuf[5] & (1 << 3)) != 0;

  // Check magnetometer data ready status
  txBuf[0] = LSM9DS1_STATUS_REG_M;
  spiTransfer(txBuf, rxBuf, 2);
  bool magDataReady = (rxBuf[1] & (1 << 0)) != 0;

  // Return true if all sensors have data ready, false otherwise
  return accelDataReady && gyroDataReady && magDataReady;
}

LSM9DS1::~LSM9DS1() {
  // Reset device
  reset();

  // Close SPI file descriptor
  close(spiFd);
}

} // namespace LSM9DS1Namespace