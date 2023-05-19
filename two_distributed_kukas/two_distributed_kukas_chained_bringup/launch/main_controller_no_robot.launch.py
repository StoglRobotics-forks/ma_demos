# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
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


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level_driver",
            default_value="info",
            description="Set the logging level of the loggers of all started nodes.",
            choices=[
                "debug",
                "DEBUG",
                "info",
                "INFO",
                "warn",
                "WARN",
                "error",
                "ERROR",
                "fatal",
                "FATAL",
            ],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level_all",
            default_value="info",
            description="Set the logging level of the loggers of all started nodes.",
            choices=[
                "debug",
                "DEBUG",
                "info",
                "INFO",
                "warn",
                "WARN",
                "error",
                "ERROR",
                "fatal",
                "FATAL",
            ],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "control_node",
            default_value="ros2_control_node_max_update_rate",
            description="Change the control node which is used.",
            choices=[
                "ros2_control_node",
                "ros2_control_node_steady_clock",
                "ros2_control_node_max_update_rate",
                "ros2_control_node_max_update_rate_sc",
                "ros2_control_node_fixed_period",
                "ros2_control_node_fixed_period_sc",
            ],
        )
    )

    # initialize arguments
    control_node = LaunchConfiguration("control_node")
    log_level_driver = LaunchConfiguration("log_level_driver")
    log_level_all = LaunchConfiguration("log_level_all")

    # Get controller manager settings
    main_robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("two_distributed_kukas_chained_bringup"),
            "controller_config",
            "no_central_robot_two_distributed_robots.yaml",
        ]
    )

    # MAIN CONTROLLER MANAGER
    # Main controller manager node
    main_control_node = Node(
        package="kuka_ros2_control_support",
        executable=control_node,
        parameters=[main_robot_controllers],
        remappings=[
            (
                "/forward_position_controller/commands",
                "/position_commands",
            ),
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            ["KukaSystemPositionOnlyHardware:=", log_level_driver],
            "--ros-args",
            "--log-level",
            log_level_all,
        ],
        output="both",
    )

    nodes = [main_control_node]

    return LaunchDescription(declared_arguments + nodes)
