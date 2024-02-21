#include <iostream>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <chrono>
#include <thread>
#include <cmath>

using namespace std;

int count=0;

int main() {
    const char* portname = "/dev/ttyACM0"; // Adjust this to your serial port
    const speed_t baudrate = B115200; // Adjust this to your baudrate

    // Open serial port
    int serial_fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) {
        cerr << "Error opening serial port" << endl;
        return 1;
    }

    // Configure serial port settings
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_fd, &tty) != 0) {
        cerr << "Error getting serial port attributes" << endl;
        return 1;
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
        return 1;
    }

    // Define the data sending rate
    const double sendingRate = 100.0; // Hz
    const auto sendingPeriod = chrono::milliseconds(static_cast<int>(1000.0 / sendingRate));

    // Main loop
    while (true) {
        // Array to send
        double dataToSend[3];
        dataToSend[0] = -0.07;
        dataToSend[1] = 0;
        dataToSend[2] = 0.9*sin(count/100.0);

        // Send data
        if (write(serial_fd, &dataToSend, sizeof(dataToSend)) < 0) {
            cerr << "Error writing to serial port" << endl;
            return 1;
        }

        // Receive data
        double receivedData[3];
        if (read(serial_fd, &receivedData, sizeof(receivedData)) < 0) {
            cerr << "Error reading from serial port" << endl;
            return 1;
        }

        // Print received data
        cout << "Received data: ";
        for (const auto& value : receivedData) {
            cout << value << " ";
        }
        cout << endl;

        count++;

        // Wait to achieve the desired sending rate
        this_thread::sleep_for(sendingPeriod);
    }

    // Close the serial port
    close(serial_fd);

    return 0;
}
