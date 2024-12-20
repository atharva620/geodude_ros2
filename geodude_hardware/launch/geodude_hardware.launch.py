# Copyright 2023 Personal Robotics Lab, University of Washington
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# Author: Ethan K. Gordon

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_package",
            default_value="geodude_hardware",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="geodude_ros2_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="geodude_hardware",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="geodude_hardware.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim",
            default_value="mock",
            description="Which sim to use: 'mock', 'isaac', or 'none'",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Whether to use sim time or not",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_frequency", 
            default_value="15.0",
            description="Publish rate of robot state publisher.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_left", 
            default_value="false",
            description="Start up left arm and optionally the left hand. Cannot start left hand without left arm.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_right", 
            default_value="false",
            description="Start up right arm and optionally the right hand. Cannot start right hand without right arm.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_left_hand", 
            default_value="false",
            description="Start up left hand. If false, make sure to update jont_state_broadcaster/empty_joints.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_right_hand", 
            default_value="false",
            description="Start up right hand. If false, make sure to update jont_state_broadcaster/empty_joints.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "move_to_initial_joint_position", 
            default_value="false",
            description="Move the joints to a default position at startup",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_vention", 
            default_value="false",
            description="Start up Vention. If false, make sure to update jont_state_broadcaster/empty_joints.",
        )
    )


    # Initialize Arguments
    controllers_package = LaunchConfiguration("controllers_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    sim = LaunchConfiguration("sim")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_left = LaunchConfiguration("use_left")
    use_left_hand = LaunchConfiguration("use_left_hand")
    use_right = LaunchConfiguration("use_right")
    use_right_hand = LaunchConfiguration("use_right_hand")
    move_to_initial_joint_position = LaunchConfiguration("move_to_initial_joint_position")
    use_vention = LaunchConfiguration("use_vention")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "sim:=",
            sim,
            " ",
            "left_arm:=",
            use_left,
            " ",
            "right_arm:=",
            use_right,
            " ",
            "left_hand:=",
            use_left_hand,
            " ",
            "right_hand:=",
            use_right_hand,
            " ",
            "move_to_initial_joint_position:=",
            move_to_initial_joint_position,
            " ",
            "use_vention:=",
            use_vention,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(controllers_package),
            "config",
            controllers_file,
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_controllers,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ],
        output="both",
        on_exit=Shutdown(),
    )

    # Given the published joint states, publish tf for the robot links and the robot description
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="both",
        parameters=[
            robot_description,
            {
                "publish_frequency": LaunchConfiguration("publish_frequency"),
            },
        ],
    )

    nodes = [
        control_node,
        rsp_node,
    ]

    actions = []

    return LaunchDescription(declared_arguments + nodes + actions)
