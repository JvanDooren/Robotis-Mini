#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class HeadServoPublisher : public rclcpp::Node {
private:
    static constexpr auto POSITION_TOPIC_NAME = "/robotis_mini/servo/head/position";
    static constexpr auto VELOCITY_TOPIC_NAME = "/robotis_mini/servo/head/velocity";
    static constexpr auto EFFORT_TOPIC_NAME = "/robotis_mini/servo/head/effort";
    static constexpr auto TOPIC_QUEUESIZE = 10;

    rclcpp::TimerBase::SharedPtr _timer;
    struct {
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr velocity;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr effort;
    } _publisher;
public:
    HeadServoPublisher() : Node("head_servo_publisher") {
        _publisher = {
            this->create_publisher<std_msgs::msg::Float64>(POSITION_TOPIC_NAME, TOPIC_QUEUESIZE),
            this->create_publisher<std_msgs::msg::Float64>(VELOCITY_TOPIC_NAME, TOPIC_QUEUESIZE),
            this->create_publisher<std_msgs::msg::Float64>(EFFORT_TOPIC_NAME, TOPIC_QUEUESIZE)
        };

        _timer = this->create_wall_timer(500ms, [this]() {
            auto message = std_msgs::msg::Float64();
            message.data = 1.0;
            RCLCPP_INFO(get_logger(), "Publishing: '%f'", message.data);
            _publisher.position->publish(message);
        });
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeadServoPublisher>());
    rclcpp::shutdown();
    return 0;
}