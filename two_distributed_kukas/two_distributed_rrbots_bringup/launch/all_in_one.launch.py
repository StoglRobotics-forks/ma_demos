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
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def delete_direct_slash_duplicate(t):
    if not (t[0] == "/" and t[1] == "/"):
        return t[0]


def prepend_slash_if_not_null(prefix):
    if not prefix:
        return ""
    ns = "/" + prefix
    # remove all occurrences of slashes that directly follow each other ("//Prefix/////Namespace//" -> "/Prefix/Namespace/")
    return "".join(
        filter(
            lambda item: item is not None,
            map(delete_direct_slash_duplicate, zip(ns, ns[1:] + " ")),
        )
    )


def generate_launch_description():
    # TODO(Manuel): find a way to propagate to controllers.yaml
    # otherwise we have to define there too
    controller_manager_name = "controller_manager"
    satellite_1_ns_name = "sub_1"
    satellite_2_ns_name = "sub_2"
    slash_controller_manager_name = prepend_slash_if_not_null(controller_manager_name)
    slash_satellite_1_ns_name = prepend_slash_if_not_null(satellite_1_ns_name)
    slash_satellite_2_ns_name = prepend_slash_if_not_null(satellite_2_ns_name)
    satellite_1_controller_manager_name = (
        slash_satellite_1_ns_name + slash_controller_manager_name
    )
    satellite_2_controller_manager_name = (
        slash_satellite_2_ns_name + slash_controller_manager_name
    )

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Start robot with fake hardware mirroring command to its states.",
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

    # initialize arguments
    prefix = LaunchConfiguration("prefix")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    listen_ip_address = LaunchConfiguration("listen_ip_address")
    listen_port = LaunchConfiguration("listen_port")

    # Get URDF via xacro
    main_robot_description_content = Command(
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
            prefix,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "controllers_file:=kuka_6dof_controllers.yaml",
            " ",
            "robot_description_package:=kuka_kr16_support",
            " ",
            "robot_description_macro_file:=kr16_2_macro.xacro",
            " ",
            "robot_name:=kuka_kr16_2",
            " ",
            "listen_ip_address:=",
            listen_ip_address,
            " ",
            "listen_port:=",
            listen_port,
            " ",
        ]
    )

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
            satellite_1_ns_name,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "controllers_file:=kuka_6dof_controllers.yaml",
            " ",
            "robot_description_package:=kuka_kr16_support",
            " ",
            "robot_description_macro_file:=kr16_2_macro.xacro",
            " ",
            "robot_name:=kuka_kr16_2",
            " ",
            "listen_ip_address:=",
            listen_ip_address,
            " ",
            "listen_port:=",
            listen_port,
            " ",
        ]
    )

    robot_satellite_2_description_content = Command(
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
            satellite_2_ns_name,
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "controllers_file:=kuka_6dof_controllers.yaml",
            " ",
            "robot_description_package:=kuka_kr16_support",
            " ",
            "robot_description_macro_file:=kr16_2_macro.xacro",
            " ",
            "robot_name:=kuka_kr16_2",
            " ",
            "listen_ip_address:=",
            listen_ip_address,
            " ",
            "listen_port:=",
            listen_port,
            " ",
        ]
    )

    # create parameter from description content
    main_robot_description = {"robot_description": main_robot_description_content}
    robot_satellite_1_description = {
        "robot_description": robot_satellite_1_description_content
    }
    robot_satellite_2_description = {
        "robot_description": robot_satellite_2_description_content
    }

    # Get controller manager settings
    main_robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("two_distributed_kukas_bringup"),
            "controller_config",
            "main_controller.yaml",
        ]
    )

    robot_controllers_satellite_1 = PathJoinSubstitution(
        [
            FindPackageShare("two_distributed_kukas_bringup"),
            "controller_config",
            "distributed_kuka_kr16_2_controller.yaml",
        ]
    )

    robot_controllers_satellite_2 = PathJoinSubstitution(
        [
            FindPackageShare("two_distributed_kukas_bringup"),
            "controller_config",
            "distributed_kuka_kr16_2_controller.yaml",
        ]
    )

    # MAIN CONTROLLER MANAGER
    # Main controller manager node
    main_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[main_robot_description, main_robot_controllers],
        remappings=[
            (
                "/forward_position_controller/commands",
                "/position_commands",
            ),
        ],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[main_robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "-c", "/controller_manager"],
    )

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        )
    )

    # SUBSYSTEMS
    # subsystem 1, satellite controller
    sub_1_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=satellite_1_ns_name,
        parameters=[robot_satellite_1_description, robot_controllers_satellite_1],
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
        output="both",
        namespace=satellite_1_ns_name,
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

    robot_controller_spawner_1 = Node(
        package="controller_manager",
        executable="spawner",
        namespace=satellite_1_ns_name,
        arguments=[
            "forward_position_controller",
            "-c",
            satellite_1_controller_manager_name,
        ],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner_1 = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner_1,
                on_exit=[robot_controller_spawner_1],
            )
        )
    )

    # SUBSYSTEMS
    # subsystem 1, satellite controller

    sub_2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=satellite_2_ns_name,
        parameters=[robot_satellite_2_description, robot_controllers_satellite_2],
        remappings=[
            (
                "/forward_position_controller/commands",
                "/position_commands",
            ),
            ("/joint_states", slash_satellite_2_ns_name + "/joint_states"),
            (
                "/dynamic_joint_states",
                slash_satellite_2_ns_name + "/dynamic_joint_states",
            ),
        ],
        output="both",
    )

    robot_state_pub_node_2 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=satellite_2_ns_name,
        output="both",
        parameters=[robot_satellite_2_description],
    )

    joint_state_broadcaster_spawner_2 = Node(
        package="controller_manager",
        executable="spawner",
        namespace=satellite_2_ns_name,
        arguments=[
            "joint_state_broadcaster",
            "-c",
            satellite_2_controller_manager_name,
        ],
    )

    robot_controller_spawner_2 = Node(
        package="controller_manager",
        executable="spawner",
        namespace=satellite_2_ns_name,
        arguments=[
            "forward_position_controller",
            "-c",
            satellite_2_controller_manager_name,
        ],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner_2 = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner_2,
                on_exit=[robot_controller_spawner_2],
            )
        )
    )

    # RVIZ
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("two_distributed_rrbots_description"),
            "rviz_config",
            "two_distributed_rrbots_all_in_on.rviz",
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    nodes = [
        main_control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        # sub_1_control_node,
        # robot_state_pub_node_1,
        # joint_state_broadcaster_spawner_1,
        # # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner_1,
        # sub_2_control_node,
        # robot_state_pub_node_2,
        # joint_state_broadcaster_spawner_2,
        # # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner_2,
        delay_rviz_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
