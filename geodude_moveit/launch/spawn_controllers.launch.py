from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch
from launch import LaunchDescription
from launch_ros.actions import Node

"""
This function reads from controller manager specified in moveit_controllers.yaml. 

The controller manager is launched by geodude_hardware's ros2 control node.

This function returns the generate_spawn_controllers_launch function from moveit_configs_utils.launches.
"""
def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("geodude", package_name="geodude_moveit").to_moveit_configs()
    controller_names = moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])
    ld = LaunchDescription()
    
    # Although joint_state_broadcasters for the hand are launched in 
    # active state by default, they don't enable hand read and write operations.
    # This means that you won't be able to see the finger joint states  
    # until the hand_effort_controllers are launched. This is done by the 
    # use of control level flags in barrett.cpp to enable the hand 
    # read write operations only when the WAM is stationary by activating 
    # the hand effort controllers that trigger the control level flags
    # enabling hand read and write operations for grasping and releasing objects.
    joint_broadcaster_controllers = ["joint_state_broadcaster_right_arm",
                                     "joint_state_broadcaster_right_hand",
                                     "joint_state_broadcaster_left_arm",
                                     "joint_state_broadcaster_left_hand",
                                     "joint_state_broadcaster_vention",]

    for controller in joint_broadcaster_controllers:
     ld.add_action(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
                output="screen",
            )
        )
    
    # The vention_trajectory_controller is launched in active state by default
    # because it doesn't block the high frequency control loop of the WAM
    for controller in controller_names:
        if controller == "vention_trajectory_controller":
            ld.add_action(
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[controller],
                        output="screen",
                    )
                )
        else:
            ld.add_action(
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[controller, "--inactive"], # spawns the controllers in inactive state with the --inactive flag
                    output="screen",
                )
            )
    return ld
