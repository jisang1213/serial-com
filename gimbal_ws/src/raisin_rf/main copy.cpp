// Copyright (c) 2020 Robotics and Artificial Intelligence Lab, KAIST
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.

#include <sched.h>

#include "raisin_rf/joy.hpp"

namespace raisin
{
namespace robot
{
namespace communication
{
extern "C"

int main(int argc, char * argv[])
{
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
