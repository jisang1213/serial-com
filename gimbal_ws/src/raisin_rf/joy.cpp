// Copyright (c) 2020 Robotics and Artificial Intelligence Lab, KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.

#include "raisin_rf/joy.hpp"

#include <asm/ioctls.h>
#include <asm/termbits.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/poll.h>

#include <string.h>
#include <unistd.h>

#include <deque>
#include <iostream>

namespace raisin
{
namespace robot
{
namespace communication
{
#define SBUS_BUFFER_SIZE 25
#define COUNT_LIMIT 3

Joy::Joy(const std::string & port)
: rclcpp::Node("joy"), receiver_thread_should_exit_(false), serial_port_fd_(-1)
{
  if (!connectSerialPort(port)) {
    rclcpp::shutdown();
  }
  createPublisher();
}

Joy::~Joy() {disconnectSerialPort();}

void Joy::disconnectSerialPort()
{
  std::cout << "Disconnect" << std::endl;
  receiver_thread_should_exit_ = true;
  close(serial_port_fd_);
}

bool Joy::connectSerialPort(const std::string & port)
{
  // Open serial port
  // O_RDWR - Read and write
  // O_NOCTTY - Ignore special chars like CTRL-C
  serial_port_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
  std::cout << "Connect to serial port" << std::endl;
  if (serial_port_fd_ == -1) {
    std::cerr << "Could not open serial port" << std::endl;
    return false;
  }
  if (!configureSerialPortForSBus()) {
    close(serial_port_fd_);
    std::cerr << "Could not set necessary configuration of serial port" << std::endl;
    return false;
  }
  return true;
}

bool Joy::configureSerialPortForSBus() const
{
  // clear config
  fcntl(serial_port_fd_, F_SETFL, 0);
  // read non blocking
  fcntl(serial_port_fd_, F_SETFL, FNDELAY);

  struct termios2 uart_config;
  /* Fill the struct for the new configuration */
  ioctl(serial_port_fd_, TCGETS2, &uart_config);

  // Output flags - Turn off output processing
  // no CR to NL translation, no NL to CR-NL translation,
  // no NL to CR translation, no column 0 CR suppression,
  // no Ctrl-D suppression, no fill characters, no case mapping,
  // no local output processing
  //
  uart_config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

  // Input flags - Turn off input processing
  // convert break to null byte, no CR to NL translation,
  // no NL to CR translation, don't mark parity errors or breaks
  // no input parity check, don't strip high bit off,
  // no XON/XOFF software flow control
  //
  uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
  //
  // No line processing:
  // echo off
  // echo newline off
  // canonical mode off,
  // extended input processing off
  // signal chars off
  //
  uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

  // Turn off character processing
  // Turn off odd parity
  uart_config.c_cflag &= ~(CSIZE | PARODD | CBAUD);

  // Enable parity generation on output and parity checking for input.
  uart_config.c_cflag |= PARENB;
  // Set two stop bits, rather than one.
  uart_config.c_cflag |= CSTOPB;
  // No output processing, force 8 bit input
  uart_config.c_cflag |= CS8;
  // Enable a non standard baud rate
  uart_config.c_cflag |= BOTHER;

  // Set custom baud rate of 100'000 bits/s necessary for sbus
  const speed_t spd = 100000;
  uart_config.c_ispeed = spd;
  uart_config.c_ospeed = spd;

  if (ioctl(serial_port_fd_, TCSETS2, &uart_config) < 0) {
    std::cerr << "could not set configuration of serial port" << std::endl;
    return false;
  }
  return true;
}


void Joy::Publish_zero_cmd(){
    for(int i=0; i<100; i++) {
        std::cout << "Reconnect USB port and Restart RF_node" << std::endl;
        sensor_msgs::msg::Joy joyMsg;
        auto now = std::chrono::high_resolution_clock::now();
        auto timestamp = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
        joyMsg.header.stamp = rclcpp::Time(timestamp.time_since_epoch().count());
        unsigned int eventNum = 8;
        unsigned int buttonNum = 2;
        joyMsg.axes.resize(eventNum);
        joyMsg.buttons.resize(buttonNum);
        for (unsigned int i = 0; i < joyMsg.axes.size(); i++) {
            joyMsg.axes[i] = 0.0;
        }
        for(unsigned int i=0; i<joyMsg.buttons.size(); i++){
            joyMsg.buttons[i] = 0;}
        joyPublisher_->publish(joyMsg);
        usleep(100000);
    }
}


void Joy::StartReceiveLoop()
{
  struct pollfd fds[1];
  fds[0].fd = serial_port_fd_;
  fds[0].events = POLLIN;
  uint8_t init_buf[10];
  while (read(serial_port_fd_, init_buf, sizeof(init_buf)) > 0) {
    // On startup, as long as we receive something, we keep reading to ensure
    // that the first byte of the first poll is the start of an SBUS message
    // and not some arbitrary byte.
    // This should help to get the framing in sync in the beginning.
    usleep(100);
  }

  std::deque<uint8_t> bytes_buf;
  while (!receiver_thread_should_exit_) {
    // Buffer to read bytes from serial port. We make it large enough to
    // potentially contain 4 sbus messages but its actual size probably does
    // not matter too much
    uint8_t read_buf[4 * kSbusFrameLength_];

    if (poll(fds, 1, kPollTimeoutMilliSeconds_) > 0) {
      if (fds[0].revents & POLLIN) {
        const ssize_t nread = read(serial_port_fd_, read_buf, sizeof(read_buf));
        //                std::cout<<nread<<std::endl;
        //Check the size of read_byte
        //If 0byte status continue, disconnect the connection and shutdown the node.

        //Initialize JoyMsg
        sensor_msgs::msg::Joy joyMsg;
        auto now = std::chrono::high_resolution_clock::now();
        auto timestamp = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
        joyMsg.header.stamp = rclcpp::Time(timestamp.time_since_epoch().count());

        unsigned int eventNum = 8;
        unsigned int buttonNum = 2;
        joyMsg.axes.resize(eventNum);
        joyMsg.buttons.resize(buttonNum);
        for (unsigned int i = 0; i < joyMsg.axes.size(); i++) {
          joyMsg.axes[i] = 0.0;
        }
          for(unsigned int i=0; i<joyMsg.buttons.size(); i++)
          { joyMsg.buttons[i] = 0;}

        if (nread <= 0) {
          zeroByteCount += 1;
        } else {
          zeroByteCount = 0;
        }
        if (COUNT_LIMIT < zeroByteCount) {
          std::cerr << "USB connection lost." << std::endl;
          disconnectSerialPort();
          break;  // Exit the loop and handle the connection loss
        }

        for (ssize_t i = 0; i < nread; i++) {
          bytes_buf.push_back(read_buf[i]);
        }

        bool valid_sbus_message_received = false;
        uint8_t sbus_msg_bytes[kSbusFrameLength_];
        while (bytes_buf.size() >= kSbusFrameLength_) {
          // Check if we have a potentially valid SBUS message
          // A valid SBUS message must have to correct header and footer byte
          // as well as zeros in the four most significant bytes of the flag
          // byte (byte 23)
          if (
            bytes_buf.front() == kSbusHeaderByte_ && !(bytes_buf[kSbusFrameLength_ - 2] & 0xF0) &&
            bytes_buf[kSbusFrameLength_ - 1] == kSbusFooterByte_)
          {
            for (uint8_t i = 0; i < kSbusFrameLength_; i++) {
              sbus_msg_bytes[i] = bytes_buf.front();
              bytes_buf.pop_front();
            }

            valid_sbus_message_received = true;
          } else {
            // If it is not a valid SBUS message but has a correct header byte
            // we need to pop it to prevent staying in this loop forever
            bytes_buf.pop_front();
            std::cout << "SBUS message framing not in sync" << std::endl;
            //                        printf("SBUS message framing not in sync");
          }

          // If not, pop front elements until we have a valid header byte
          while (!bytes_buf.empty() && bytes_buf.front() != kSbusHeaderByte_) {
            bytes_buf.pop_front();
          }
        }

        if (valid_sbus_message_received) {
          unsigned int channels[16];
          float msg[8] = {0};
            int control[2] = {0};
          channels[0] = ((sbus_msg_bytes[1] | sbus_msg_bytes[2] << 8) & 0x07FF);
          channels[1] = ((sbus_msg_bytes[2] >> 3 | sbus_msg_bytes[3] << 5) & 0x07FF);
          channels[2] =
            ((sbus_msg_bytes[3] >> 6) | (sbus_msg_bytes[4] << 2) |
            (sbus_msg_bytes[5] << 10) & 0x07FF);
          channels[3] = ((sbus_msg_bytes[5] >> 1) | (sbus_msg_bytes[6] << 7) & 0x07FF);
          channels[4] = ((sbus_msg_bytes[6] >> 4) | (sbus_msg_bytes[7] << 4)) & 0x07FF;
          channels[5] = ((sbus_msg_bytes[7] >> 7) | (sbus_msg_bytes[8] << 1) | (sbus_msg_bytes[9] << 9)) & 0x07FF;
          channels[6] = (((uint16_t)sbus_msg_bytes[9] >> 2) |((uint16_t)sbus_msg_bytes[10] << 6)) & 0x07FF;
          channels[7] = (((uint16_t)sbus_msg_bytes[10] >> 5) |((uint16_t)sbus_msg_bytes[11] << 3)) & 0x07FF;
          channels[8] =(((uint16_t)sbus_msg_bytes[12]) | ((uint16_t)sbus_msg_bytes[13] << 8)) & 0x07FF;
          //joy msg mapping

          for (int i = 0; i < 4; ++i) {
            if (abs((int)channels[i] - 992) < 40) {
              msg[i] = 0;
            } else if ((int)channels[i] - 992 > 810) {
              msg[i] = 1;
            } else if ((int)channels[i] - 992 < -810) {
              msg[i] = -1;
            } else {
              msg[i] = (float)((int)channels[i] - 992) / 810;
            }
          }
          if (channels[4] > 1500) {///stand_up_down
            msg[4] = -1;
          } else if (channels[4] < 500) {
            msg[4] = 1;
          }
          if (channels[5] > 1500) {///estop
            msg[5] = -1;
          } else if (channels[5] < 500) {
            msg[5] = 1;
          }
            if (channels[6]>1500) {
                control[0] = 1;
                } ///control select
            else if (channels[6]<500) {
                control[0] = 0;
            }
             if (channels[7]>1500) {
                 control[1] = 1;
             }
            else if (channels[7]<500) {
                control[1] = 0;
            }    ///load trigger

          if (
            channels[0] == 0 && channels[1] == 0 && channels[2] == 0 &&
            channels[3] == 0 & channels[4] == 0 & channels[5] == 0)
          {
            for (int i = 0; i < 8; ++i) {
              msg[i] = 0;
            }
          }    ///for wireless failsafe

          joyMsg.axes[0] = -1 * msg[0];
          joyMsg.axes[1] = msg[1];
          joyMsg.axes[3] = -1 * msg[3];
          joyMsg.axes[4] = msg[2];
          joyMsg.axes[7] = msg[4];
          joyMsg.axes[6] = msg[5];
          joyMsg.buttons[0] = control[0];
          joyMsg.buttons[1] = control[1];
          joyPublisher_->publish(joyMsg);
        }
      }
    }
  }
  disconnectSerialPort();
}

void Joy::createPublisher()
{
  rclcpp::QoS qos(rclcpp::KeepLast(1));

  joyPublisher_ = this->create_publisher<sensor_msgs::msg::Joy>("joy", qos);
}

}  // namespace communication

}  // namespace robot

}  // namespace raisin
