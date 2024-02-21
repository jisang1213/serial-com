#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <cmath>

using namespace std;

// Function to configure the serial port
int configureSerialPort(const char *portname, int baudrate) {

    // Open serial port
    int serial_fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) {
        cerr << "Error opening serial port" << endl;
    }

    // Configure serial port settings
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_fd, &tty) != 0) {
        cerr << "Error getting serial port attributes" << endl;
    }
    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit characters
    tty.c_iflag &= ~IGNBRK;                         // Disable break processing
    tty.c_lflag = 0;                                // No signaling characters, no echo, no canonical processing
    tty.c_oflag = 0;                                // No remapping, no delays
    tty.c_cc[VMIN]  = 0;                            // Read doesn't block
    tty.c_cc[VTIME] = 5;                            // 0.5 seconds read timeout
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // Shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);                // Ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);              // No parity
    tty.c_cflag &= ~CSTOPB;                         // 1 stop bit
    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        cerr << "Error setting serial port attributes" << endl;
    }
    return serial_fd;
}

//define SerialNode class
class SerialNode : public rclcpp::Node {
public:
    SerialNode(int serialPort) : Node("serial_node"), serialPort(serialPort) {
        // Subscribe to the input topic
        sub_ = create_subscription<geometry_msgs::msg::Vector3>(
            "/command_topic", 10, std::bind(&SerialNode::inputCallback, this, std::placeholders::_1));

        // Advertise the output topic
        pub_ = create_publisher<geometry_msgs::msg::Vector3>("/return_topic", 10);
    }

private:
    //this function gets called whenever the node receives data from subcribed topic.
    void inputCallback(const geometry_msgs::msg::Vector3::SharedPtr input_msg) {
        // Array to send
        double dataToSend[3];
        dataToSend[0] = input_msg->x; 
        dataToSend[1] = input_msg->y; 
        dataToSend[2] = input_msg->z; 

        //write command to MCU
        // Send data
        if (write(serialPort, &dataToSend, sizeof(dataToSend)) < 0) {
            cerr << "Error writing to serial port" << endl;
        }
        //get MCU response
        double receivedData[3];
        if (read(serialPort, &receivedData, sizeof(receivedData)) < 0) {
            cerr << "Error reading from serial port" << endl;
        }

        // Print received data
        cout << "Received data: ";
        for (const auto& value : receivedData) {
            cout << value << " ";
        }
        cout << endl;

        //Publish the real joint angles to the return topic
        auto output_msg = std::make_unique<geometry_msgs::msg::Vector3>();
        output_msg->x = receivedData[0];
        output_msg->y = receivedData[1];
        output_msg->z = receivedData[2];

        // Publish the processed message to the output topic
        pub_->publish(std::move(output_msg));
    }

    int serialPort;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_;
};

int main(int argc, char** argv) {

    //open serial port
    const char *serialPortName = "/dev/ttyACM0";  // Serial port device path
    int baudRate = B115200;  // Baud rate

    int serialPort = configureSerialPort(serialPortName, baudRate);
    if (serialPort == -1) {
        return 1;
    }

    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);

    // Create an instance of the SerialNode class, passing serialPort as argument
    auto serial_node = std::make_shared<SerialNode>(serialPort);

    // Spin to process callbacks
    rclcpp::spin(serial_node);

    // Shutdown the ROS 2 node
    rclcpp::shutdown();

    //close the serial port
    close(serialPort);

    return 0;
}