#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string.h>
using namespace std;

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <sys/time.h>

#include <chrono>

#include "CppConsoleTable/CppConsoleTable.hpp"

// for convenience
using ConsoleTable = samilton::ConsoleTable;

static chrono::time_point<chrono::steady_clock> time_point_first_sample;

static unsigned long expectedBytesPerSecond = 0;
static unsigned long measuredBytesPerSecond = 0;

static unsigned long nr_received_samples = 0;
static constexpr unsigned int sample_size_bytes = 4;
static constexpr unsigned int expected_nr_samples = 100000;

static void outputResults() {
    ConsoleTable table(1, 1, samilton::Alignment::centre);

    // headers
	table[0][0] = "expected B/s";
    table[0][1] = "measured B/s";
    table[0][2] = "efficiency";
    table[0][3] = "delta";
    table[0][4] = "delta rate B/s";

    table[1][0] = expectedBytesPerSecond;
    table[1][1] = measuredBytesPerSecond;
    table[1][2] = "0.0%";
    table[1][3] = 0;
    table[1][4] = 0;

	cout << table;
}

static void handleRemoteTimestamp(unsigned long remote_micros) {
    if (nr_received_samples == 0) {
        time_point_first_sample = chrono::steady_clock::now();
    }
    nr_received_samples++;
    if (nr_received_samples % 1000 == 0) cout << "samples " << nr_received_samples << endl;

    if (nr_received_samples == expected_nr_samples) {
        auto local_delta_millis = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - time_point_first_sample).count();
        // average
        //cout << "local delta ms " << local_delta_millis << endl;

        measuredBytesPerSecond = (1000 * nr_received_samples * sample_size_bytes) / local_delta_millis;
        outputResults();
    }

    //cout << "Got timestamp " << remote_micros << endl;
}

static speed_t toSpeed(unsigned int baud) {
    switch (baud) {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 500000:
        return B500000;
    case 576000:
        return B576000;
    case 921600:
        return B921600;
    case 1000000:
        return B1000000;
    case 1152000:
        return B1152000;
    case 1500000:
        return B1500000;
    case 2000000:
        return B2000000;
    case 2500000:
        return B2500000;
    case 3000000:
        return B3000000;
    case 3500000:
        return B3500000;
    case 4000000:
        return B4000000;
    default: 
        return -1;
    }
}

int main(int argc, char** argv) {
    // open the serial port passed as argument and output to file
    if (argc != 3) {
        cout << "This program should be called with arguments {serial-port} {baudrate}" << endl;
        cout << "For instance " << argv[0] << " /dev/rfcomm1 115200" << endl;
        return 0;
    }

    auto serial_port = open(argv[1], O_RDONLY);
    if (serial_port < 0) {
        cout << "Failure to open " << argv[1] << " due to " << errno << endl;
        return 1;
    }

    // Create new termios struc, we call it 'tty' for convention
    struct termios tty;

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
        return 1;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON; // Disable canonical
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be argv[2]
    stringstream ss(argv[2]);
    unsigned int baud;
    if( ss >> baud )
        cout << "baud is: " << baud << endl;
    else
        cout << "error" << endl;

    cfsetspeed(&tty, toSpeed(baud));

    expectedBytesPerSecond = baud / 8;

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        cout << "Error " << errno << " from tcsetattr: " << strerror(errno) << endl;
        return 1;
    }

    // Allocate memory for read buffer, set size according to your needs
    char read_buf[4096];
    int num_bytes = 0;
    int offset = 0;
    int total_read = 0;
    int received_nr_bytes = 0;
    unsigned long timestamp;
    while (1) {
        num_bytes = read(serial_port, &read_buf[offset], sizeof(read_buf)-offset);
        /*for (int i=0; i<num_bytes; i++) {
            cout << "b[" << i << "]=0x" << std::hex << ((unsigned int)read_buf[i] & 0x000000FF) << std::dec << endl;
        }*/
        received_nr_bytes += num_bytes;
        total_read += num_bytes;
        if (num_bytes > 0) {
            cout << "Got " << num_bytes << " bytes" << endl;
            cout << "Received total " << received_nr_bytes << " bytes" << endl;
        }
        while (total_read >= sample_size_bytes) {
            timestamp = (read_buf[3] << 24) & 0xFF000000 | (read_buf[2] << 16) & 0x00FF0000 | (read_buf[1] << 8) & 0x0000FF00 | (read_buf[0] << 0) & 0x000000FF;
            handleRemoteTimestamp(timestamp);
            total_read -= sample_size_bytes;
            memmove(read_buf, &read_buf[4], total_read);
        }
        offset = total_read;
    }

    close(serial_port);
    return 0; // success
}
