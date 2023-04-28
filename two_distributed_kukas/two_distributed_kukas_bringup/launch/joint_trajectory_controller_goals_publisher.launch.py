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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "goals_file",
            default_value="6dof_joint_trajectory_controller_goals.yaml",
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
            choices=[
                "6dof_joint_trajectory_controller_goals.yaml",
                "one_central_one_distributed_kuka_6dof_joint_trajectory_controller_goals.yaml",
                "two_distributed_kukas_6dof_joint_trajectory_controller_goals.yaml",
            ],
        )
    )

    # init arguments
    goals_file = LaunchConfiguration("goals_file")

    position_goals = PathJoinSubstitution(
        [
            FindPackageShare("two_distributed_kukas_bringup"),
            "controller_config/joint_trajectory_goals",
            goals_file,
        ]
    )

    goal_publisher_node = Node(
        package="ros2_controllers_test_nodes",
        executable="publisher_joint_trajectory_controller",
        name="publisher_joint_trajectory_controller",
        parameters=[position_goals],
        output="both",
    )

    nodes = [
        goal_publisher_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
