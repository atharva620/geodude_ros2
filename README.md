# geodude_ros2
## About
This repository contains the ROS2 packages for the Geodude project. These packages include `geodude_hardware`, `geodude_moveit`, and `geodude_description`. The `geodude_hardware` package contains two hardware interfaces for controlling Barrett WAM arms, Barrett Hands, and Vention Lead Screw Actuators based on the [`ros2_control`](https://control.ros.org/rolling/index.html) framework. The `geodude_moveit` package contains a customized `MoveIt2` configuration and the `geodude_description` package contains the URDF and meshes.

## Hardware Setup
The Geodude setup consists of a pair of [Barrett WAM](https://advanced.barrett.com/wam-arm-1) (7-DOF arms) mounted vertically on linear actuators with (BH8-282) [Barrett Hands](https://advanced.barrett.com/barretthand) (4-DOF, dexterous) as end effectors. The WAMs and Barrett Hands are controlled by [`libbarrett`](https://github.com/personalrobotics/libbarrett), which is a C++ library that runs Barrett Technology products either real-time or non-real-time/low-latency. The Geodude project uses the low-latency version of `libbarrett`.

## How to run `geodude_hardware` on the real system with `'sim:=none'`

**Instructions tested on an external control computer with Ubuntu 22.04**

1. Make sure you have followed the README at [our fork of libbarrett](https://github.com/personalrobotics/libbarrett) to install the `libbarrett` library. The `libbarrett` library should be installed in `/usr/local/lib` and the `libbarrett` headers should be installed in `/usr/local/include`.

2. Ensure the workspace builds (e.g. `colcon build`) and `uname -r` output is the same as the kernel version in the `libbarrett` installation i.e. `low-latency`
   
3. If running on the real system, make sure the robot's left arm is loaded as `can11` and right arm is loaded as `can10`. The CAN port can be checked in `geodude_hardware/config/left_wam7w/default.conf` and/or `geodude_hardware/config/right_wam7w/default.conf`.

4. If the above check fails, see the [PCAN Linux Drive Manual](https://www.peak-system.com/fileadmin/media/linux/files/PCAN-Driver_for_Linux_eng_7.x.pdf) for info about assigning devid, and edit `/etc/modprob.d/pcan.conf` with `options pcan assign=devid` to load it as the device number.

5. Now power up the WAM by turning the inverter switch on using the remote control or manually. Power up the Vention Lead Screw Actuators by pressing the power button on the MachineMotion controller, releasing the e-stop and then pressing the reset button. 

6. Make sure the e-stops are released on the control pendants and WAM/s are zero-d: At this point make sure the CAN bus is reset with the correct bitrate: `sudo reset_can.sh` (libbarrett should install this in `/bin`).

7. Run `ros2 run geodude_hardware zero_barrett left` and/or `ros2 run geodude_hardware zero_barrett right`
   
7. The above commands will prompt you to Shift-Idle and zero the WAM/s. The user must manually bring the arms to home position if that is not already done. Optionally, you may also zero the Vention Lead Screw Actuators by running `ros2 run geodude_hardware zero_vention`

8. Launch the hardware interface for controlling both the WAM arms, Barrett Hands, and the Vention Lead Screw Actuators:
```
ros2 launch geodude_hardware geodude_hardware.launch.py 'sim:=none' 'use_left:=true' 'use_right:=true' 'use_left_hand:=true' 'use_right_hand:=true' 'move_to_initial_joint_position:=true' 'use_vention:=true'
```

9. The above command will prompt you to Shift-Activate the WAM. Once you press Shift-Activate on the control pendant, the hardware interface will start and the controllers for the WAMs and Barrett Hands will be loaded in an unconfigured state by the controller manager.
   
10.  By default, the hardware interface starts the system in stopped state i.e. it makes the WAMs and Barrett Hands hold their positions. This is also the state in which the `ros2_control` state machine ends up when a ROS2 controller is unloaded to prevent uncontrolled falling.

11.  With the hardware interface running, you can then run the `MoveIt2` Demo. This will change the state of the controllers to inactive and load the `move_group` node for planning and executing trajectories for the WAMs and Barrett Hands. To launch the `MoveIt2` demo:
```
ros2 launch geodude_moveit demo.launch.py
```

12. The above command will launch the `move_group` node and `RViz`. Note that this will also launch the joint trajectory controllers for both arms and hands in `inactive state` i.e. the controllers will not follow the trajectory commands sent by `MoveIt2`. To confirm the state of the controllers: `ros2 control list_controllers`
    
13. Before sending any trajectory commands to move the WAM arm/s, deactivate the hand controller if not already inactive: (For more info on why this is necessary, see the `geodude_hardware` README)
```
ros2 control set_controller_state left_hand_effort_controller inactive
```
```
ros2 control set_controller_state right_hand_effort_controller inactive
```
14. Activate the arm controllers:
```
ros2 control set_controller_state left_arm_position_controller active
```
```
ros2 control set_controller_state right_arm_position_controller active
```
15.  Now you can plan and execute collision-free trajectories for the WAMs using the `MoveIt2`. The WAMs and Barrett Hands will follow the trajectory commands sent by the planner.
    
15.  To grasp an object with the Barrett Hands, you need to activate the relevant hand controller/s:
```
ros2 control set_controller_state left_hand_effort_controller active
```
```
ros2 control set_controller_state right_hand_effort_controller active
```
17.  Repeat 13, 14, and 15, 16 to switch between controlling the WAMs and Barrett Hands as needed. ***WARNING: You should not move the arm while any of the hand controllers are active***

18. To exit the hardware interface, press `Ctrl+C` in the terminal where the hardware interface was launched. This will stop the hardware interface and move the WAM arm to its home position **without collision checking**. Hence you should only press `ctrl+c` when the end effector/J7 joint is clear of any obstacles and does not require a significant amount of rotation to reach the home position. This is especially important for the Barrett Hands as they have a large workspace and can easily collide with the first few links of the WAM arm.

*Note: the WAM must be zero-ed in order for `on_configure` call to be successful*

*Note: You must manually press Shift-Activate for `on_activate` call to be successful.*

## How to run `geodude_hardware` with Isaac Sim `'sim:=isaac'`
1. Follow the instructions in [Isaac Sim - Joint Control](https://docs.omniverse.nvidia.com/isaacsim/latest/ros_tutorials/tutorial_ros_manipulation.html) to setup the Isaac Sim environment and the ROS2 bridge (tested on ROS2 Humble).
2. Launch `geodude_hardware` with the `sim` argument set to `isaac`:
```
ros2 launch geodude_hardware geodude_hardware.launch.py 'use_left:=true' 'use_right:=true' 'use_left_hand:=true' 'use_right_hand:=true' 'sim:=isaac'
```
3. Launch the `MoveIt2` demo:
```
ros2 launch geodude_moveit demo.launch.py
```
4. Press the Play button in Isaac Sim to start the simulation, plan to any random valid pose in `MoveIt2`, and execute the trajectory. Expect to see the simulated robot move to the planned pose in Isaac Sim.
5. This option is dependent on the `topic_based_ros2_control` ROS2 pacakge. Make sure you have it installed. For additional information, see [How to command issac simulated robot with MoveIt2](https://moveit.picknik.ai/main/doc/how_to_guides/isaac_panda/isaac_panda_tutorial.html) tutorial.

## How to run `geodude_hardware` with the mock hardware interface `'sim:=mock'`
1. Launch `geodude_hardware` with the `sim` argument set to `mock`:
```
ros2 launch geodude_hardware geodude_hardware.launch.py 'sim:=mock'
```
2. Launch the `MoveIt2` demo:
```
ros2 launch geodude_moveit demo.launch.py
```
3. Plan to any random valid pose in `MoveIt2`, and execute the trajectory. Expect to see the robot move to the planned pose in RViz.
4. For more information on the mock hardware interface, see the corresponding [ros2_control - Mock Components](https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/mock_components_userdoc.html) documentation.

## Contributing to this repository

1. Format your C++ code with `ament_clang_format --reformat [directory]`
A standard way to do this is to run the formatter in `geodude_hardware`:
```
$ cd geodude_ros2/geodude_hardware
$ ament_clang_format --reformat .
```

1. Make sure that all tests pass: `colcon test`
This checks formatting (flake8 for python that can be fixed with the `autopep8` command and `clang_format` for cpp which can be fixed as above)
This also runs all GTests.

1. Push your changes to a different branch and make a pull request to `main`

Make sure to document all testing (real robot or otherwise) as well as what was changed.
