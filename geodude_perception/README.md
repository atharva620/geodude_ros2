# Geodude Perception

This module contains code for general Object Detection and Pose Estimation using YOLO World and Foundation Pose.

## Get Started

Install isaac-ros docker: https://nvidia-isaac-ros.github.io/concepts/docker_devenv/index.html#development-environment

Run docker:

`$ISAAC_ROS_WS/src/isaac_ros_common/scripts/run_dev.sh -d $ISAAC_ROS_WS -a "-v /home/$user:/home/$user"`

export ROS_DOMAIN_ID:

`export ROS_DOMAIN_ID=your_id`

Install Dependencies in docker

`sudo apt-get install -y ros-humble-isaac-ros-foundationpose && sudo apt install ros-humble-vision-msgs-rviz-plugins && pip install ultralytics`

cd into geodude folder:

`cd /home/user/geodude_ws`

build and source

`colcon build --packages-select geodude_perception && source install/setup.bash`


## Usage:

launch object pose estimation

`ros2 launch geodude_perception track_object.launch.py`

launch argument:

`texture_file_path` : path to texture file \
`mesh_file_path` : path to mesh file \
`input_image_topic` : image topic \
`input_depth_topic` : depth topic \
`input_camera_info_topic`: camera info topic \
`tf_name` : name of the object in the tf tree \
`launch_rviz` : launch rviz visualization \
`classes`: object names pass to the object detection model Ex. ['mustard']