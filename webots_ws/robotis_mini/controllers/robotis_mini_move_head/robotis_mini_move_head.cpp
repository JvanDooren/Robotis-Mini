// File:          robotis_mini_move_forward.cpp
// Date:
// Description:
// Author:
// Modifications:

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


// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  Motor *neckMotor = robot->getMotor(NeckMotorId_17);
  Motor *rightHipMotor = robot->getMotor(RightHipMotorId_7);
  Motor *leftHipMotor = robot->getMotor(LeftHipMotorId_8);
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    neckMotor->setPosition(1.0);
    rightHipMotor->setPosition(1.0);
    leftHipMotor->setPosition(1.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
