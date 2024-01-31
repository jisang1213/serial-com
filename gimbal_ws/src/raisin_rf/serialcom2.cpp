#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>

typedef union{
    double d[3];
    uint8_t c[24];
} data;

data senddata;
data receivedata;

int count = 1;

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
        std::cout << "Received data: " << buffer << std::endl;
    }
}

int main() {
    const char *serialPortName = "/dev/cu.usbmodem367C344F31321";  // Change this to your actual serial port
    int baudRate = B9600;  // Change this to your desired baud rate

    int serialPort = configureSerialPort(serialPortName, baudRate);
    if (serialPort == -1) {
        return 1;
    }

    while(1){
        std::cout << count;

        //command
        senddata.d[0] = 0.0;   //*check type
        senddata.d[1] = 0.0;
        senddata.d[2] = 0.0;

        //write command to MCU
        writeSerial(serialPort, (char*)senddata.c, sizeof(senddata.c));
        usleep(200);
        std::cout << ":  Sent data" << std::endl;

        readSerial(serialPort, (char*)receivedata.c, sizeof(receivedata.c));
        std::cout << receivedata.d[0] << std::endl;
        std::cout << receivedata.d[1] << std::endl;
        std::cout << receivedata.d[2] << std::endl;

        usleep(500000);
        count++;
    }

    close(serialPort);

    return 0;
}