#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cmath>

#include <chrono>


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
int readSerial(int serialPort, char *buffer, int bufferSize) {
    int bytesRead = read(serialPort, buffer, bufferSize);
    if (bytesRead > 0) {
        //buffer[bytesRead] = '\0'; // Null-terminate the received data
        std::cout << "Received data: " << std::endl;
        return 0;
    }
    else{
        std::cout << "Not received. " << std::endl;
        return -1;
    }
}

int main() {
    const char *serialPortName = "/dev/ttyACM0";  // Change this to your actual serial port
    int baudRate = B9600;  // Change this to your desired baud rate

    int serialPort = configureSerialPort(serialPortName, baudRate);
    if (serialPort == -1) {
        return 1;
    }

    while(1){
        //command
        senddata.d[0] = 0.1;   //*check type
        senddata.d[1] = 0.2;
        senddata.d[2] = 0.3;

        //write command to MCU
        writeSerial(serialPort, (char*)senddata.c, sizeof(senddata.c));
        std::cout << count << ":  Sent data" << std::endl;
        std::cout << senddata.d[0] << std::endl;
        std::cout << senddata.d[1] << std::endl;
        std::cout << senddata.d[2] << std::endl;
        usleep(10);
        

        if(readSerial(serialPort, (char*)receivedata.c, sizeof(receivedata.c)) == 0){
            std::cout << receivedata.d[0] << std::endl;
            std::cout << receivedata.d[1] << std::endl;
            std::cout << receivedata.d[2] << std::endl;
        }

        usleep(10000);
        count++;
    }

    close(serialPort);

    return 0;
}