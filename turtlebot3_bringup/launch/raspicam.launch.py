#!/usr/bin/env python3
#
# Copyright 2020 ROBOTIS CO., LTD.
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
#
# Authors: Ryan Shim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_param_dir = LaunchConfiguration(
        'camera_param_dir',
        default=os.path.join(
            get_package_share_directory('turtlebot3_bringup'),
            'param',
            'camera.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_param_dir',
            default_value=camera_param_dir,
            description='Full path to camera parameter file to load'),

        Node(
            package='raspicam2',
            node_executable='raspicam2_node',
            parameters=[camera_param_dir],
            output='screen'),
    ])
