// Copyright (c) 2020 Robotics and Artificial Intelligence Lab, KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.

#include <sched.h>

#include "raisin_rf/joy.hpp"
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace raisin
{
namespace robot
{
namespace communication
{
extern "C"

int main(int argc, char * argv[])
{

  const char *portName = "/dev/ttyACM0";  // Replace with your actual serial port

  // Open the serial port
  int serialPort = open(portName, O_RDWR);
  if (serialPort == -1) {
      perror("Error opening serial port");
      return 1;
  }

  // Configure the serial port
  struct termios tty;
  if (tcgetattr(serialPort, &tty) != 0) {
      perror("Error getting serial port attributes");
      close(serialPort);
      return 1;
  }
  
  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  cfsetospeed(&tty, B115200);  // Set the baud rate to 115200
  cfsetispeed(&tty, B115200);

  tty.c_cflag |= (CLOCAL | CREAD);  // Enable receiver and ignore modem control lines
  tty.c_cflag &= ~PARENB;           // Disable parity bit
  tty.c_cflag &= ~CSTOPB;           // Use one stop bit
  tty.c_cflag &= ~CSIZE;            // Clear the mask for data bits
  tty.c_cflag |= CS8;               // Set 8 data bits
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Raw input

  if (tcsetattr(serialPort, TCSANOW, &tty) != 0) {
      perror("Error setting serial port attributes");
      close(serialPort);
      return 1;
  }

  // Send data
  float senddata[3] = {3.14f, 1.23f, 2.54f};
  write(serialPort, senddata, 3*sizeof(float));

  // Read data
  float receivedata[3];
  int bytesRead = read(serialPort, receivedata, sizeof(receivedata));
  if (bytesRead > 0) {
      std::cout << "Received: " << receivedata[0] << std::endl;
  } else {
      perror("Error reading from serial port");
  }

  // Close the serial port
  close(serialPort);

  // set cpu processor
  cpu_set_t mask;
  CPU_ZERO(&mask);
  CPU_SET(6, &mask);
  int result = sched_setaffinity(0, sizeof(mask), &mask);

  // run process
  rclcpp::init(argc, argv);
  std::cout << "start" << std::endl;

  Joy joy = Joy("/dev/ftdi");
  joy.StartReceiveLoop();
  joy.Publish_zero_cmd();
  std::cout << "Node finished" << std::endl;
  rclcpp::shutdown();
  return 0;
}

}  // namespace communication

}  // namespace robot

}  // namespace raisin
