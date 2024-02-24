# Copyright 2020 ros2_control Development Team
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
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution,LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    arg_mock = DeclareLaunchArgument(
            "use_mock_hardware",
            default_value='false',
            description="Start robot with mock hardware mirroring command to its states.",
        )
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    urdf_path = PathJoinSubstitution([FindPackageShare("bot_base"), "urdf", "bot_base.urdf.xacro"])
    robot_description_content = ParameterValue(Command(['xacro ', urdf_path," ","use_mock_hardware:=",use_mock_hardware]), value_type=str)
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description_content},
                     PathJoinSubstitution([FindPackageShare("bot_differential_drive"),"config","controller.yaml"])],
        output="both")
    
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description_content}],
        remappings=[("/bot_base_controller/cmd_vel", "/cmd_vel")])

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"])

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["bot_base_controller", "--controller-manager", "/controller_manager"])

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        arg_mock,
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)