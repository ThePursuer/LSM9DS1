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

  // Enable interrupts (optional)
  txBuf[0] = LSM9DS1_INT1_CTRL;
  txBuf[1] = 0x01; // Interrupt on new data available
  spiTransfer(txBuf, rxBuf, 2);
  
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

} // namespace LSM9DS1Namespace