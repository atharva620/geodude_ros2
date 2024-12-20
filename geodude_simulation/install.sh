# Install script for geodude_simulation package

# Install the orbbic camera package
cd /home/helenwang/geodude_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
/lib/systemd/systemd-udevd --daemon
bash install_udev_rules.sh
udevadm control --reload-rules && udevadm trigger

mv /home/helenwang/geodude_ws/src/geodude_ros2/patches/librcl_logging_spdlog.so /opt/ros/humble/lib

cd /home/helenwang/geodude_ws/
colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release --packages-skip geodude_hardware moveit2_tutorials

# Install the isaac tutorials package
cd /home/helenwang/IsaacSim-ros_workspaces
source /opt/ros/humble/setup.bash
apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
apt install python3-colcon-common-extensions
cd humble_ws
rosdep init
rosdep update
apt-get update
rosdep install -i --from-path src --rosdistro iron -y
source install/local_setup.bash



export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/isaac-sim/exts/omni.isaac.ros2_bridge/humble/lib
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/helenwang/geodude_ws/src/geodude_ros2/geodude_simulation/fastdds.xml
