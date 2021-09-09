#!/usr/bin/env python

# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Webots and the controller."""

import os
import launch

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

from webots_ros2_core.utils import ControllerLauncher
from webots_ros2_core.webots_launcher import WebotsLauncher

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Webots
    webots = WebotsLauncher(
        #world=os.path.join(get_package_share_directory('robotis_mini_description'), 'worlds', 'ros_example.wbt')
        world=os.path.join('./worlds', 'empty.wbt')
    )

    # Controller node
    # synchronization = launch.substitutions.LaunchConfiguration('synchronization', default=False)
    # controller = ControllerLauncher(
    #    package='head_node',
    #    executable='head_publisher',
    #    parameters=[{'synchronization': synchronization}],
    #    output='screen'
    #)

    # urdf for state publisher
    robot_description_filename = 'robotis_mini.urdf.xacro'
    print("robot_description_filename : {}".format(robot_description_filename))
    xacro = os.path.join(
      get_package_share_directory('robotis_mini_description'), 'urdf',
      robot_description_filename)
    print("xacro : {}".format(xacro))
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    xacro_path = LaunchConfiguration('xacro_path', default='{}'.format(xacro))
    print("xacro_path : {}".format(xacro_path))

    gui = LaunchConfiguration('gui', default='true')

    return launch.LaunchDescription([
        webots,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description':Command(['xacro',' ', xacro_path])
            }]),
        Node(
            condition=UnlessCondition(gui),
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'),
        Node(
            condition=IfCondition(gui),
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])