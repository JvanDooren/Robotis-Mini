# Robotis-Mini

Robotis Mini Arduino IDE code, ROS2 interface

# Setup ROS2 development machine

1. First install Ubuntu 20.04 Focal
2. Install ROS2 by following https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/
3. Install ROS2 xacro by `sudo apt install ros-$ROS_DISTRO-xacro`
4. Install VSCode
5. Setup Robotis Mini by following https://emanual.robotis.com/docs/en/software/arduino_ide/
6. Install colcon by `sudo apt install python3-colcon-common-extensions`
7. Install rosdep2 by `sudo apt install python3-rosdep2 && rosdep update`
8. Download MeshLab for meshes by following https://www.meshlab.net/#download. This can be used to fine-tune values in the xacro files by loading a mesh. See http://gazebosim.org/tutorials?tut=inertia&cat=build_robot
   Filters - Quality Measure and Computations - Compute Geometric Measures

# Building ROS2 packages

1. Change to the ros2_ws folder `cd Robotis-Mini/ros2_ws`
2. Check before building that all dependencies have been met `rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y`
3. Build all packages in the workspace folder: `colcon build`
4. To let ROS2 find our own built packages, add the workspace folder to the ROS2 path `. install/setup.bash`
5. Run the bt210_bridge `ros2 run bt210_bridge bt210bridge`
6. Convert the .xacro to .proto file for webots `ros2 run webots_ros2_importer xacro2proto --disable-mesh-optimization --output=../webots_ws/robotis_mini/protos/RobotisMini.proto src/robotis_mini_description/urdf/robotis_mini.urdf.xacro` Note: conversion works from the installed xacro files, not from the source repo, so a `colcon build` is required to install the files. Center of mass specified as 0 0 0 in xacro causes it not being picked up into the .proto file, causing a warning.

# Setup ROS2 simulator

We're going to utilize Webots, so follow https://github.com/cyberbotics/webots_ros2/wiki/Getting-Started

1. Install `sudo apt-get install ros-$ROS_DISTRO-webots-ros2`
2. To check whether it is installed correctly, run `ros2 launch webots_ros2_demos armed_robots.launch.py`
3. Start the webots simulator and load the world `webots_ws/robotis_mini/worlds/empty.wbt`

A nice tutorial can be found at `https://www.youtube.com/watch?v=jU_FD1_zAqo&list=PLt69C9MnPchkP0ZXZOqmIGRTOch8o9GiQ`

# Setup RQT

This is a graphical user interface that can query and control ROS2 interfaces

1. Install `sudo apt install ~nros-foxy-rqt*`
2. Launch `rqt`

# Launch the WeBots with our robot

Before we can start webots, install joint_state_publisher `sudo apt-get install ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-joint-state-publisher-gui`
A convenience python scipt has been created to launch Webots and nodes. To make the nodes discoverable:
`cd Robotis-Mini/webots_ws/robotis_mini`
`. ../../ros2_ws/install/setup.bash`
`ros2 launch robotis.launch.py`

OR

`ros2 launch webots_ros2_core robot_launch.py world:={absolute-path-to}/Robotis-Mini/webots_ws/robotis_mini/worlds/empty.wbt`

Once Webots is running, discover nodes: `ros2 node list`
Once Webots is running, discover services: `ros2 service list`
To discover what a service topic supports, query it: `ros2 service call [/service] [double tab]`
Every service can now be queried for possibilities: `ros2 interface show [double tab]`
Rtq is a visual tool that shows the same.

## Set BT-210 baudrate

The BT-210 is default set to 57600 baud, which is too slow for relaying state.
Follow the instructions at https://emanual.robotis.com/docs/en/parts/communication/bt-210/ to set to a higher baudrate.
Use the define in the serial2serial ino file to set the BT-210 communication baudrate to 57600, then upload the ino.
After changing the baudrate by AT commands, enforce the setting by power-cycling the BT-210 or 'ATZ'. Once the setting is active, this new baudrate (1382400) will be the fixed rate for opening Serial2 on Arduino.

### Effective transmission rate

After some tests, it appears that the max rate the BT-210 can withstand internally (without overflowing buffers and loosing bytes) is around 22000 bytes/s, which is roughly 176000 baud. Essentially, there needs to be a minimum delay of 175us between every Serial2.write call to prevent the BT-210 from loosing bytes. So, although the set baudrate of 1382400 baud is far from achievable, this setting still gives us around 175us of time do do something else in the loop (which should be perfect to get the status of all Dynamixels).

## Arduino source

https://github.com/chcbaram/OpenCM9.04/tree/master/hardware/robotis/OpenCM9.04/cores/arduino

## Mathmatical docs

https://cris.maastrichtuniversity.nl/ws/portalfiles/portal/33237631/c6330.pdf
