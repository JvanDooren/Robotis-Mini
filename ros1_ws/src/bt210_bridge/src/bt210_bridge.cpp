#include "ros/ros.h"
//#include "std_msgs/String.h"
#include "bt210_bridge/Xl320StateMsg.h"

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

static constexpr auto NODE_NAME = "BT-210_Bridge";
static constexpr auto TOPIC_NAME = "/robotis-mini/servo/17";
static constexpr auto TOPIC_BUFFERSIZE = 1000;

static constexpr auto SERIALPORT_DEVICE = "/dev/rfcomm1";
static constexpr auto SERIALPORT_BAUDRATE = 1382400;

using bt210_bridge::Xl320StateMsg;

class BT210SerialPort {
private:
    int _bt210Fd;

private:
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

    void _open() noexcept {
        _bt210Fd = open(SERIALPORT_DEVICE, O_RDONLY);
        if (_bt210Fd < 0) {
            ROS_FATAL("Failure to open %s due to %d", SERIALPORT_DEVICE, errno);
            return;
        }

        // Create new termios struc, we call it 'tty' for convention
        struct termios tty;

        // Read in existing settings, and handle any error
        if(tcgetattr(_bt210Fd, &tty) != 0) {
            ROS_FATAL("Error %d from tcgetattr: %s", errno, strerror(errno));
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
            ROS_FATAL("Error %d from tcsetattr: %s", errno, strerror(errno));
            close(_bt210Fd);
            return;
        }
    }

public:
    BT210SerialPort() {
        _open();
    }

    bool isOpen() {
      return _bt210Fd >= 0;
    }
};

/**
 * This node opens the Bluetooth dongle (registered at /dev/rfcomm1)
 * and connects to the BT-210. Once connected, it reads all data
 * and translates that to a ROS topic message after which it is published
 */
int main(int argc, char **argv)
{
    /**
    * The ros::init() function needs to see argc and argv so that it can perform
    * any ROS arguments and name remapping that were provided at the command line.
    * For programmatic remappings you can use a different version of init() which takes
    * remappings directly, but for most command-line programs, passing argc and argv is
    * the easiest way to do it.  The third argument to init() is the name of the node.
    *
    * You must call one of the versions of ros::init() before using any other
    * part of the ROS system.
    */
      //TODO: pass serial port and speed via arguments
    ros::init(argc, argv, NODE_NAME);

    BT210SerialPort port;

    if (!port.isOpen()) {
        return -1;
    }

    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
    ros::NodeHandle n;

    /**
    * The advertise() function is how you tell ROS that you want to
    * publish on a given topic name. This invokes a call to the ROS
    * master node, which keeps a registry of who is publishing and who
    * is subscribing. After this advertise() call is made, the master
    * node will notify anyone who is trying to subscribe to this topic name,
    * and they will in turn negotiate a peer-to-peer connection with this
    * node.  advertise() returns a Publisher object which allows you to
    * publish messages on that topic through a call to publish().  Once
    * all copies of the returned Publisher object are destroyed, the topic
    * will be automatically unadvertised.
    *
    * The second parameter to advertise() is the size of the message queue
    * used for publishing messages.  If messages are published more quickly
    * than we can send them, the number here specifies how many messages to
    * buffer up before throwing some away.
    */
    ros::Publisher servo17_pub = n.advertise<Xl320StateMsg>(TOPIC_NAME, TOPIC_BUFFERSIZE);

    // run at 10Hz (10 times per second)
    ros::Rate loop_rate(10);

    /**
    * A count of how many messages we have sent. This is used to create
    * a unique string for each message.
    */
    while (ros::ok())
    {
        /**
        * This is a message object. You stuff it with data, and then publish it.
        */
        Xl320StateMsg msg;
        msg.ID = 17;

        /**
        * The publish() function is how you send messages. The parameter
        * is the message object. The type of this object must agree with the type
        * given as a template parameter to the advertise<>() call, as was done
        * in the constructor above.
        */
        servo17_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}
