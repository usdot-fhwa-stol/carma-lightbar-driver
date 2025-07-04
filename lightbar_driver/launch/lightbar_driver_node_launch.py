# Copyright (C) 2022 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from carma_ros2_utils.launch.get_current_namespace import GetCurrentNamespace

import os

'''
This file is can be used to launch the lightbar_driver.
  Though in carma-platform it may be launched directly from the base launch file.
'''

def generate_launch_description():

    # Declare the log_level launch argument
    log_level = LaunchConfiguration('log_level')
    declare_log_level_arg = DeclareLaunchArgument(
        name ='log_level', default_value='WARN')

    # Args for driver
    auth_config_file = LaunchConfiguration('auth_config_file')
    declare_auth_config_file = DeclareLaunchArgument(name = 'auth_config_file', default_value = '/opt/carma/vehicle/calibration/lightbar/auth_config.yaml', description="File containing Username and Password for the Lightbar IP")

    # Get parameter file path
    param_file_path = os.path.join(
        get_package_share_directory('lightbar_driver'), 'config/parameters.yaml')

    # Launch node(s) in a carma container to allow logging to be configured
    container = ComposableNodeContainer(
        package='carma_ros2_utils',
        name='lightbar_driver_container',
        namespace=GetCurrentNamespace(),
        executable='carma_component_container_mt',
        composable_node_descriptions=[

            # Launch the core node(s)
            ComposableNode(
                    package='lightbar_driver',
                    plugin='lightbar_driver::LightBarApplication',
                    name='lightbar_driver_node',
                    extra_arguments=[
                        {'use_intra_process_comms': True},
                        {'--log-level' : log_level }
                    ],
                    parameters=[auth_config_file, param_file_path],
                    remappings=[
                        ("set_lights", "lightbar/set_lights"),
                    ],
            ),
        ],

    )

    return LaunchDescription([
        declare_log_level_arg,
        declare_auth_config_file,
        container
    ])
