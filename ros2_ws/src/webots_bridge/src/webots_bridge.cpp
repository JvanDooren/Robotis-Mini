/**
 * This node subscribes to the robotis mini nodes and transforms them into webots calls
 * This targets the Webots simulation
 */
 #include <memory>

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// the name of the motor directly relates to the name in the proto file under 'RotationalMotor'
static constexpr char NeckMotorId_17[] = "neck_joint";

static constexpr char LeftShoulderMotorId_2[] = "l_shoulder_joint";
static constexpr char LeftBicepsMotorId_4[] = "l_biceps_joint";
static constexpr char LeftElbowMotorId_6[] = "l_elbow_joint";

static constexpr char RightShoulderMotorId_1[] = "r_shoulder_joint";
static constexpr char RightBicepsMotorId_3[] = "r_biceps_joint";
static constexpr char RightElbowMotorId_5[] = "r_elbow_joint";

static constexpr char LeftHipMotorId_8[] = "l_hip_joint";
static constexpr char LeftThighMotorId_10[] = "l_thigh_joint";
static constexpr char LeftKneeMotorId_12[] = "l_knee_joint";
static constexpr char LeftAnkleMotorId_14[] = "l_ankle_joint";
static constexpr char LeftFootMotorId_16[] = "l_foot_joint";

static constexpr char RightHipMotorId_7[] = "r_hip_joint";
static constexpr char RightThighMotorId_9[] = "r_thigh_joint";
static constexpr char RightKneeMotorId_11[] = "r_knee_joint";
static constexpr char RightAnkleMotorId_13[] = "r_ankle_joint";
static constexpr char RightFootMotorId_15[] = "r_foot_joint";

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
private:
    //TODO: move
    static constexpr auto POSITION_TOPIC_NAME = "/robotis_mini/servo/head/position";
    static constexpr auto VELOCITY_TOPIC_NAME = "/robotis_mini/servo/head/velocity";
    static constexpr auto EFFORT_TOPIC_NAME = "/robotis_mini/servo/head/effort";
    static constexpr auto TOPIC_QUEUESIZE = 10;

    struct {
        struct {
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity;
            rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr effort;
        } subscription;
        Motor* motor;
    } Node;

    struct {
        Node neck;
    } _nodes;

    Robot robot;
public:
    MinimalSubscriber() : Node("webots_bridge_subscriber") {
        _nodes = {
            {
                {
                    this->create_subscription<std_msgs::msg::Float64>(POSITION_TOPIC_NAME, TOPIC_QUEUESIZE, [this](const std_msgs::msg::Float64::SharedPtr msg) {
                        RCLCPP_INFO(get_logger(), "Setting position: '%f'", msg->data);
                        motor->setPosition(msg->data);
                    }),
                },
                robot.getMotor(NeckMotorId_17);
            },
        };
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
