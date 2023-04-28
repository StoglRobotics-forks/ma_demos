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
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

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

    # Get controller manager settings
    main_robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("two_distributed_kukas_bringup"),
            "controller_config",
            "main_controller.yaml",
        ]
    )

    # MAIN CONTROLLER MANAGER
    # Main controller manager node
    main_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[main_robot_controllers],
        remappings=[
            (
                "/forward_position_controller/commands",
                "/position_commands",
            ),
        ],
        output="both",
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
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)
