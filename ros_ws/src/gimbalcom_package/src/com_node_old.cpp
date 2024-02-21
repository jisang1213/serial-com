#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

//union to reinterpret double array as char array for serial communication
typedef union{
    double d[3];
    uint8_t c[24];
} data;

//**check if Vector3 uses floats(4bytes) or double(8btyes)

data senddata;
data receivedata;

// Function to configure the serial port
int configureSerialPort(const char *port, int baudRate) {
    int serialPort = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    
    if (serialPort == -1) {
        std::cerr << "Error opening serial port." << std::endl;
        return -1;
    }

    struct termios serialConfig;
    tcgetattr(serialPort, &serialConfig);

    cfsetispeed(&serialConfig, baudRate);
    cfsetospeed(&serialConfig, baudRate);

    serialConfig.c_cflag &= ~PARENB;   // No parity
    serialConfig.c_cflag &= ~CSTOPB;   // 1 stop bit
    serialConfig.c_cflag &= ~CSIZE;
    serialConfig.c_cflag |= CS8;       // 8 data bits
    serialConfig.c_cflag |= CREAD;     // Enable reading
    serialConfig.c_cflag |= CLOCAL;    // Ignore modem control lines

    tcsetattr(serialPort, TCSANOW, &serialConfig);
    return serialPort;
}

// Function to write data to the serial port
void writeSerial(int serialPort, const char *data, int dataSize) {
    write(serialPort, data, dataSize);
}

// Function to read data from the serial port
void readSerial(int serialPort, char *buffer, int bufferSize) {
    int bytesRead = read(serialPort, buffer, bufferSize);
    if (bytesRead > 0) {
        //buffer[bytesRead] = '\0'; // Null-terminate the received data
        std::cout << "Received data: " << std::endl;
    }
    else{
        std::cout << "Not received" << std::endl;
    }
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
        //process command data
        senddata.d[0] = input_msg->x;   //*check type
        senddata.d[1] = input_msg->y;
        senddata.d[2] = input_msg->z;

        //write command to MCU
        writeSerial(serialPort, (char*)senddata.c, sizeof(senddata.c));
        //usleep(500);
        //get MCU response
        readSerial(serialPort, (char*)receivedata.c, sizeof(receivedata.c));
        //std::cout << receivedata.c[0] << receivedata.c[1] << receivedata.c[2] << receivedata.c[3] << receivedata.c[4] << receivedata.c[5] << receivedata.c[6] << receivedata.c[7] << std::endl;
        std::cout << receivedata.d[0] << " " << receivedata.d[1] << " " << receivedata.d[2] << std::endl;

        //Publish the real joint angles to the return topic
        auto output_msg = std::make_unique<geometry_msgs::msg::Vector3>();
        output_msg->x = receivedata.d[0];
        output_msg->y = receivedata.d[1];
        output_msg->z = receivedata.d[2];

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