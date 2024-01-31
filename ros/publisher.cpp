#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

class MyPublisherNode : public rclcpp::Node {
public:
  MyPublisherNode() : Node("my_publisher_node") {
    publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("my_topic", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&MyPublisherNode::publishMessage, this));
  }

private:
  void publishMessage() {
    auto message = geometry_msgs::msg::Vector3();

    // Increment the count for the next message
    count++;

    // Increment the values of the Vector3 message every millisecond
    message.x = static_cast<double>(count);
    message.y = static_cast<double>(count + 1);
    message.z = static_cast<double>(count + 2);

    // Publish the message
    publisher_->publish(message);
  }

  int count = 0;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
