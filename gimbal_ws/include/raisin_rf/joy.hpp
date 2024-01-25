// Copyright (c) 2020 Robotics and Artificial Intelligence Lab, KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.

#ifndef RAISIN_RF__JOY_HPP_
#define RAISIN_RF__JOY_HPP_

#include <atomic>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace raisin
{
namespace robot
{
namespace communication
{
class Joy : public rclcpp::Node
{
public:
  Joy();
  Joy(const std::string & port);
  virtual ~Joy();

  /// run rf
  void run();
  void StartReceiveLoop();
  void Publish_zero_cmd();

protected:
  bool connectSerialPort(const std::string & port);
  void disconnectSerialPort();

private:
  /// setup rf

  bool processSbusData(const unsigned char * buffer, int length, float * msg);
  bool configureSerialPortForSBus() const;
  /// create ros publisher
  void createPublisher();

private:
  static constexpr int kSbusFrameLength_ = 25;
  static constexpr uint8_t kSbusHeaderByte_ = 0x0F;
  static constexpr uint8_t kSbusFooterByte_ = 0x00;
  static constexpr int kPollTimeoutMilliSeconds_ = 500;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joyPublisher_;
  int serial_port_fd_;  // variable for serial port allocation
  std::atomic_bool receiver_thread_should_exit_;
  unsigned int zeroByteCount;
};

}  // namespace communication

}  // namespace robot

}  // namespace raisin

#endif  // RAISIN_RF__JOY_HPP_
