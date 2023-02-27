#include <iostream>
#include <string>
#include <cstdlib>
#include <thread>
#include <unistd.h>
#include <signal.h>

#include "LSM9DS1.hpp"

// Global flag for handling kill signal
volatile bool g_running = true;

// Signal handler for handling kill signal
void signalHandler(int signal) {
  g_running = false;
}

int main(int argc, char* argv[]) {
  // Set up signal handler for handling kill signal
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  // Parse command-line options
  std::string spiDevice = "/dev/spidev0.0";
  int option;
  while ((option = getopt(argc, argv, "d:")) != -1) {
    switch (option) {
      case 'd':
        spiDevice = optarg;
        break;
      default:
        std::cerr << "Usage: " << argv[0] << " [-d spi-device]" << std::endl;
        return 1;
    }
  }

  // Create instance of LSM9DS1 driver
  LSM9DS1Namespace::LSM9DS1 sensor(spiDevice);
  std::cout << "sensor is " << sensor.verify() << std::endl;

  // Loop until kill signal is received
  while (g_running) {
    // Read accelerometer and gyro data from sensor
    auto data = sensor.readSensorDataScaled();

    // Print accelerometer and gyro data
    std::cout << "Accelerometer (g): X=" << data.accelX << ", Y=" << data.accelY << ", Z=" << data.accelZ << std::endl;
    std::cout << "Gyroscope (dps): X=" << data.gyroX << ", Y=" << data.gyroY << ", Z=" << data.gyroZ << std::endl;

    // Wait for a short time before reading again
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Exit gracefully
  return 0;
}