/**
 * This node subscribes to the robotis mini nodes and transforms them into BT210 calls
 * This targets the actual robot
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "bt210_interface/msg/xl320_state_msg.hpp"

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

using bt210_interface::msg::Xl320StateMsg;
using namespace std::chrono_literals;

class BT210Bridge : public rclcpp::Node {
private:
    int _bt210Fd;
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<Xl320StateMsg>::SharedPtr _publisher;

    static constexpr auto NODE_NAME = "BT210_Bridge";
    static constexpr auto TOPIC_NAME = "/robotis_mini/servo/servo_17";
    static constexpr auto TOPIC_QUEUESIZE = 10;

    static constexpr auto SERIALPORT_DEVICE = "/dev/rfcomm1";
    static constexpr auto SERIALPORT_BAUDRATE = 1382400;

public:
    BT210Bridge() : Node(NODE_NAME) {
        _open();
        if (_bt210Fd < 0) {
            throw std::runtime_error("Failure to open serial port");
        }
        _publisher = this->create_publisher<Xl320StateMsg>(TOPIC_NAME, TOPIC_QUEUESIZE);
        _timer = this->create_wall_timer(500ms, [this]() {
            auto message = Xl320StateMsg();
            message.id = 17;
            RCLCPP_INFO(get_logger(), "Publishing state of servo: '%d'", message.id);
            _publisher->publish(message);
        });
    }

private:
    static speed_t toSpeed(unsigned int baud) noexcept {
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

    void _open() noexcept {
        _bt210Fd = open(SERIALPORT_DEVICE, O_RDONLY);
        if (_bt210Fd < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failure to open %s due to %d", SERIALPORT_DEVICE, errno);
            return;
        }

        // Create new termios struc, we call it 'tty' for convention
        struct termios tty;

        // Read in existing settings, and handle any error
        if(tcgetattr(_bt210Fd, &tty) != 0) {
            RCLCPP_FATAL(this->get_logger(), "Error %d from tcgetattr: %s", errno, strerror(errno));
            close(_bt210Fd);
            return;
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

        cfsetspeed(&tty, toSpeed(SERIALPORT_BAUDRATE));

        // Save tty settings, also checking for error
        if (tcsetattr(_bt210Fd, TCSANOW, &tty) != 0) {
            RCLCPP_FATAL(this->get_logger(), "Error %d from tcsetattr: %s", errno, strerror(errno));
            close(_bt210Fd);
            return;
        }
    }
};

/**
 * This node opens the Bluetooth dongle (registered at /dev/rfcomm1)
 * and connects to the BT-210. Once connected, it reads all data
 * and translates that to a ROS topic message after which it is published
 */
int main(int argc, char * argv[]) {
    //TODO: pass serial port and speed via arguments
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BT210Bridge>());
    rclcpp::shutdown();
    return 0;
}
