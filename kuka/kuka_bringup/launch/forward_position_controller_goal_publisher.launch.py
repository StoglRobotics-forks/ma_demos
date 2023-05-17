# Copyright 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
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
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def create_nodes_to_launch(context, *args, **kwargs):

    publish_mode = LaunchConfiguration("publish_mode").perform(context)

    if publish_mode == "normal":
        goals_file = "kuka_6dof_forward_position_controller_goals.yaml"
    else:
        goals_file = "kuka_6dof_chained_forward_position_controller_goals.yaml"

    position_goals = PathJoinSubstitution(
        [
            FindPackageShare("kuka_bringup"),
            "controller_config",
            goals_file,
        ]
    )

    forard_position_goal_publisher = Node(
        package="ros2_controllers_test_nodes",
        executable="publisher_forward_position_controller",
        name="publisher_forward_position_controller",
        parameters=[position_goals],
        output="both",
    )

    return [forard_position_goal_publisher]


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_mode",
            default_value="normal",
            description="Set which controller gets activated by default",
            choices=["normal", "chained"],
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=create_nodes_to_launch)]
    )
