#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
This module starts all the screen sessions to run the geodude demo.
"""

import asyncio
import argparse
import getpass
import os
import sys

parser = argparse.ArgumentParser()
parser.add_argument(
    "--sim",
    default="mock",
    help=(
        "`real`, `mock` (default `real`). `real` executes all commands for running "
        "the geodude code on the robot, `mock` uses dummy perception and real motion code "
        "with a simulated robot."
    ),
)
parser.add_argument(
    "-t",
    "--termination_wait_secs",
    default=3,
    help="How long (secs) to wait for the code within screens to terminate (default, 3)",
)
parser.add_argument(
    "-l",
    "--launch_wait_secs",
    default=0.3,
    help=(
        "How long (secs) to wait between running code in screens. Not that "
        "too low of a value can result in commands getting rearranged. (default, 0.3)"
    ),
)
parser.add_argument(
    "-c",
    "--close",
    action="store_true",
    help="If set, only terminate the code in the screens.",
)
parser.add_argument(
    "--enable_components",
    nargs="+",
    default=["left", "right", "left_hand", "right_hand"],
    help="Comma separated list of components to enable (default: all 4 components)",
)

parser.add_argument(
    "--real_domain_id",
    default=10,
    type=int,
    help=(
        "If sim is set to real, export this ROS_DOMAIN_ID before running code in "
        "every screen session. (default: 10)"
    ),
)

def check_pwd_is_colcon_workspace() -> str:
    """
    Check that the script is being run from the top-level colcon workspace.
    Return the absolute path to the current directory.
    """
    # The below are a smattering of directories and files that should exist in the
    # top-level colcon workspace.
    dirs_to_check = ["src", "build", "install", "log"]
    files_to_check = [ # TODO: files to check
    ]
    for dir_to_check in dirs_to_check:
        if not os.path.isdir(dir_to_check):
            print(
                f"ERROR: This script must be run from the top-level colcon workspace. Could not find `{dir_to_check}`",
                file=sys.stderr,
            )
            sys.exit(1)
    for file_to_check in files_to_check:
        if not os.path.isfile(file_to_check):
            print(
                f"ERROR: This script must be run from the top-level colcon workspace. Could not find `{file_to_check}`",
                file=sys.stderr,
            )
            sys.exit(1)

    # Return the absolute path to the current directory
    return os.path.abspath(".")


async def get_existing_screens():
    """
    Get a list of active screen sessions.

    Adapted from
    https://serverfault.com/questions/886405/create-screen-session-in-background-only-if-it-doesnt-already-exist
    """
    proc = await asyncio.create_subprocess_shell(
        "screen -ls", stdout=asyncio.subprocess.PIPE
    )
    stdout, _ = await proc.communicate()
    existing_screens = [
        line.split(".")[1].split("\t")[0].rstrip()
        for line in stdout.decode("utf-8").splitlines()
        if line.startswith("\t")
    ]
    return existing_screens


async def execute_command(screen_name: str, command: str, indent: int = 8) -> None:
    """
    Execute a command in a screen.
    """
    global sudo_password  # pylint: disable=global-statement
    indentation = " " * indent
    printable_command = command.replace("\003", "SIGINT")
    print(f"# {indentation}`{printable_command}`")
    if command != "\003":
        command += "\n"
    await asyncio.create_subprocess_shell(
        f"screen -S {screen_name} -X stuff '{command}'"
    )
    await asyncio.sleep(args.launch_wait_secs)
    if command.startswith("sudo"):
        if sudo_password is None:
            sudo_password = getpass.getpass(
                prompt=f"# {indentation}Enter sudo password: "
            )
        await asyncio.create_subprocess_shell(
            f"screen -S {screen_name} -X stuff '{sudo_password}\n'"
        )
        await asyncio.sleep(args.launch_wait_secs)
    elif command.startswith("ssh"):
        ssh_password = getpass.getpass(prompt=f"# {indentation}Enter ssh password: ")
        await asyncio.create_subprocess_shell(
            f"screen -S {screen_name} -X stuff '{ssh_password}\n'"
        )
        await asyncio.sleep(args.launch_wait_secs)


async def main(args: argparse.Namespace, pwd: str) -> None:
    """
    Start the geodude demo.

    Args:
        args: The command-line arguments.
        pwd: The absolute path to the current directory.
    """

    # pylint: disable=too-many-branches, too-many-statements
    # This is meant to be a flexible function, hence the many branches and statements.
    # pylint: disable=redefined-outer-name
    # That is okay in this case.

    print(
        "################################################################################"
    )
    if not args.close:
        print(f"# Starting the geodude demo in **{args.sim}**")
        print("# Prerequisites / Notes:")
        print("#     1. Be in the top-level of your colcon workspace")
        print(
            "#     2. Your workspace should be built (e.g., `colcon build --symlink-install`)"
        )
        if args.sim == "real":
            # TODO: add some warnings for running in real robot
            pass
    else:
        print(f"# Terminating the geodude demo in **{ args.sim}**")
    print(
        "################################################################################"
    )
    
    launched_components = " ".join([f"\'use_{component}:=true\'" for component in args.enable_components])
    launched_components += " ".join([f"\'use_{component}:=false\'" for component in args.disable_components])
    
    if args.sim == "mock":
        screen_sessions = {
            "camera": [
                "ros2 launch orbbec_camera femto_bolt.launch.py camera_name:=front_camera device_num:=2 usb_port:=2-3 enable_colored_point_cloud:=true",
                "ros2 launch orbbec_camera femto_bolt.launch.py camera_name:=top_camera usb_port:=2-2 device_num:=2 enable_colored_point_cloud:=true"
            ],
            "moveit": [
                "ros2 launch geodude_moveit demo.launch.py"
            ],
            "apriltag_detector": [
                "ros2 run apriltag_ros apriltag_node --ros-args \
                    -r image_rect:=/front_camera/color/image_raw \
                        -r camera_info:=/front_camera/color/camera_info \
                        --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml"
            ],
        }
        close_commands = {}
    elif args.sim == "real":
        screen_sessions = {
            "camera": [
                "ros2 launch orbbec_camera femto_bolt.launch.py camera_name:=front_camera device_num:=2 usb_port:=2-3 enable_colored_point_cloud:=true",
                "ros2 launch orbbec_camera femto_bolt.launch.py camera_name:=top_camera usb_port:=2-2 device_num:=2 enable_colored_point_cloud:=true"
            ],
            "moveit": [
                "ros2 launch geodude_moveit demo.launch.py"
            ],
            "apriltag_detector": [
                "ros2 run apriltag_ros apriltag_node --ros-args \
                    -r image_rect:=/front_camera/color/image_raw \
                        -r camera_info:=/front_camera/color/camera_info \
                        --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_36h11.yaml"
            ],
        }
        close_commands = {}

    initial_close_commands = ["\003"]
    initial_start_commands = [
        f"cd {pwd}",
        "conda deactivate",
        "source install/setup.zsh",
    ]
    
    initial_start_commands.append(f"export ROS_DOMAIN_ID={args.real_domain_id}")
    
    for screen_name, commands in screen_sessions.items():
        screen_sessions[screen_name] = initial_start_commands + commands
        if screen_name not in close_commands:
            close_commands[screen_name] = []
        close_commands[screen_name] = (
            initial_close_commands + close_commands[screen_name]
        )
    
        # Determine which screens are already running
    print("# Checking for existing screen sessions")
    terminated_screen = False
    existing_screens = await get_existing_screens()
    for screen_name in screen_sessions:
        if screen_name in existing_screens:
            print(f"#    Found session `{screen_name}`: ")
            for command in close_commands[screen_name]:
                await execute_command(screen_name, command)
            terminated_screen = True
        elif not args.close:
            print(f"#    Creating session `{screen_name}`")
            await asyncio.create_subprocess_shell(f"screen -dmS {screen_name}")
            await asyncio.sleep(args.launch_wait_secs)

    print(
        "################################################################################"
    )

    if not args.close:
        # Sleep for a bit to allow the screens to terminate
        if terminated_screen:
            print(f"# Waiting {args.termination_wait_secs} secs for code to terminate")
            await asyncio.sleep(args.termination_wait_secs)
            print(
                "################################################################################"
            )

        # Start the screen sessions
        print("# Starting geodude code")
        for screen_name, commands in screen_sessions.items():
            print(f"#     `{screen_name}`")
            for command in commands:
                await execute_command(screen_name, command)
        print(
            "################################################################################"
        )

        print("# Done! Next steps:")
        print(
            "#     1. Check individual screens to verify code is working as expected."
        )
        if args.sim == "real":
            print("#     2. Release the e-stop button, shift-activate to enable the robot if you haven't.")


if __name__ == "__main__":
    # Get the arguments
    args = parser.parse_args()
    args.disable_components = [s for s in ["left", "right", "left_hand", "right_hand"] \
        if s not in args.enable_components]
    # Check args
    if args.sim not in ["real", "mock"]:
        raise ValueError(
            f"Unknown sim value {args.sim}. Must be one of ['real', 'mock']."
        )
    
    # Ensure the script is not being run as sudo. Sudo has a different screen
    # server and may have different versions of libraries installed.
    if os.geteuid() == 0:
        print(
            "ERROR: This script should not be run as sudo. Run as a regular user.",
            file=sys.stderr,
        )
        sys.exit(1)

     # Check that the script is being run from the top-level colcon workspace
    pwd = check_pwd_is_colcon_workspace()
    
    # Run the main function
    sudo_password = None  # pylint: disable=invalid-name
    asyncio.run(main(args, pwd))

    # Return success
    sys.exit(0)