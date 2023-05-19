# Copyright 2023 Stogl Robotics Consulting UG (haftungsbeschränkt)
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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ma_demos_launch_helpers import prepend_slash_if_not_null, select_kuka_robot


def create_nodes_to_launch(context, *args, **kwargs):
    controller_manager_name = "controller_manager"
    satellite_1_ns_name = "sub_1"
    slash_controller_manager_name = prepend_slash_if_not_null(controller_manager_name)
    slash_satellite_1_ns_name = prepend_slash_if_not_null(satellite_1_ns_name)
    satellite_1_controller_manager_name = (
        slash_satellite_1_ns_name + slash_controller_manager_name
    )

    # initialize arguments
    robot = select_kuka_robot(LaunchConfiguration("robot_type").perform(context))
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    control_node = LaunchConfiguration("control_node")
    listen_ip_address = LaunchConfiguration("listen_ip_address")
    listen_port = LaunchConfiguration("listen_port")
    log_level_driver = LaunchConfiguration("log_level_driver")
    log_level_all = LaunchConfiguration("log_level_all")
    origin = LaunchConfiguration("origin")

    # SUB 1
    robot_satellite_1_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("kuka_ros2_control_support"),
                    "urdf",
                    "common_kuka.xacro",
                ]
            ),
            " ",
            "prefix:=",
            satellite_1_ns_name + "_",
            " ",
            "origin:=",
            origin,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "controllers_file:=kuka_6dof_controllers.yaml",
            " ",
            "robot_description_package:=",
            robot["robot_description_package"],
            " ",
            "robot_description_macro_file:=",
            robot["robot_description_macro_file"],
            " ",
            "robot_name:=",
            robot["robot_name"],
            " ",
            "listen_ip_address:=",
            listen_ip_address,
            " ",
            "listen_port:=",
            listen_port,
            " ",
        ]
    )
    robot_satellite_1_description = {
        "robot_description": robot_satellite_1_description_content
    }
    robot_controllers_satellite_1 = PathJoinSubstitution(
        [
            FindPackageShare("two_distributed_kukas_chained_bringup"),
            "controller_config",
            "sub_1_controller.yaml",
        ]
    )

    sub_1_control_node = Node(
        package="kuka_ros2_control_support",
        executable=control_node,
        namespace=satellite_1_ns_name,
        parameters=[robot_satellite_1_description, robot_controllers_satellite_1],
        arguments=[
            "--ros-args",
            "--log-level",
            ["KukaSystemPositionOnlyHardware:=", log_level_driver],
            "--ros-args",
            "--log-level",
            log_level_all,
        ],
        remappings=[
            (
                "/forward_position_controller/commands",
                "/position_commands",
            ),
            ("/joint_states", slash_satellite_1_ns_name + "/joint_states"),
            (
                "/dynamic_joint_states",
                slash_satellite_1_ns_name + "/dynamic_joint_states",
            ),
        ],
        output="both",
    )

    robot_state_pub_node_1 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=satellite_1_ns_name,
        output="both",
        parameters=[robot_satellite_1_description],
    )

    joint_state_broadcaster_spawner_1 = Node(
        package="controller_manager",
        executable="spawner",
        namespace=satellite_1_ns_name,
        arguments=[
            "joint_state_broadcaster",
            "-c",
            satellite_1_controller_manager_name,
        ],
    )

    nodes = [
        sub_1_control_node,
        robot_state_pub_node_1,
        joint_state_broadcaster_spawner_1,
    ]

    return nodes


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            default_value="kuka_kr5",
            description="The robot which should be launched.",
            choices=["kuka_kr5", "kuka_kr16_2"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "listen_ip_address",
            default_value="172.20.19.101",
            description="The ip address on of your device on which is listend.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "listen_port",
            default_value="49152",
            description="The port on which is listend.",
        )
    )
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
            "origin",
            default_value='"0 0 0"',
            description="Change the robots origin.",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=create_nodes_to_launch)]
    )
