#include <iostream>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>

typedef union{
    double d[3];
    uint8_t c[24];
} data;

int main() {
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
    data senddata = {.d = {3.14, 1.23, 2.54}};
    write(serialPort, senddata.c, sizeof(senddata));

    sleep(1);

    // Read data
    data receivedata;
    int bytesRead = read(serialPort, receivedata.c, sizeof(receivedata));
    if (bytesRead > 0) {
        std::cout << "Received: " << receivedata.c[0] << std::endl;
    } else {
        perror("Error reading from serial port");
    }

    // Close the serial port
    close(serialPort);

    return 0;
}