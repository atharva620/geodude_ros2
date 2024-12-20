# Copyright (c) 2024, Personal Robotics Lab
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer, Node, SetRemap
from launch_ros.descriptions import ComposableNode
import getpass

GEODUDE_WS_PATH = '/home/' + getpass.getuser() + '/geodude_ws/src/geodude_ros2'
MESH_FILE_NAME = GEODUDE_WS_PATH + '/geodude_perception/meshes/Mustard/textured_simple.obj'
TEXTURE_MAP_NAME = GEODUDE_WS_PATH + '/geodude_perception/meshes/Mustard/texture_map.png'
REFINE_ENGINE_NAME = GEODUDE_WS_PATH + '/geodude_perception/models/foundationpose/refine_trt_engine.plan'
SCORE_ENGINE_NAME = GEODUDE_WS_PATH + '/geodude_perception/models/foundationpose/score_trt_engine.plan'


def generate_launch_description():
    """Generate launch description for testing relevant nodes."""
    rviz_config_path = GEODUDE_WS_PATH + '/geodude_perception/config/perception.rviz'

    launch_args = [
        DeclareLaunchArgument(
            'mesh_file_path',
            default_value=MESH_FILE_NAME,
            description='The absolute file path to the mesh file'),

        DeclareLaunchArgument(
            'texture_path',
            default_value=TEXTURE_MAP_NAME,
            description='The absolute file path to the texture map'),

        DeclareLaunchArgument(
            'refine_engine_file_path',
            default_value=REFINE_ENGINE_NAME,
            description='The absolute file path to the refine trt engine'),

        DeclareLaunchArgument(
            'score_engine_file_path',
            default_value=SCORE_ENGINE_NAME,
            description='The absolute file path to the score trt engine'),

        DeclareLaunchArgument(
            'image_topic',
            default_value='/image_rect',
            description='The topic on which the image is published on'),

        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera_info',
            description='The topic on which the camera info is published on'),

        DeclareLaunchArgument(
            'depth_topic',
            default_value='/depth',
            description='The topic on which the depth image is published on'),

        DeclareLaunchArgument(
            'image_input_topic',
            default_value='/front_camera/color/image_raw',
            description='The topic to be republished with different format'),

        DeclareLaunchArgument(
            'camera_info_input_topic',
            default_value='/front_camera/color/camera_info',
            description='The topic to be republished with different format'),

        DeclareLaunchArgument(
            'depth_input_topic',
            default_value='/front_camera/depth/image_raw',
            description='The topic to be republished with different format'),

        DeclareLaunchArgument(
            'image_height',
            default_value='720',
            description='The height of the mask generated from the bounding box'),

        DeclareLaunchArgument(
            'image_width',
            default_value='1280',
            description='The width of the mask generated from the bounding box'),

        DeclareLaunchArgument(
            'launch_bbox_to_mask',
            default_value='True',
            description='Flag to enable bounding box to mask converter'),

        DeclareLaunchArgument(
            'launch_rviz',
            default_value='False',
            description='Flag to enable Rviz2 launch'),

        DeclareLaunchArgument(
            'launch_tracking',
            default_value='False',
            description='Flag to enable tracking'),

        DeclareLaunchArgument(
            'tf_frame',
            default_value='fp_object',
            description='The tf frame to use for the camera'),

        DeclareLaunchArgument(
            'classes',
            default_value="['bottle', 'can']",
            description='The classes to detect'),
    ]

    mesh_file_path = LaunchConfiguration('mesh_file_path')
    texture_path = LaunchConfiguration('texture_path')
    refine_engine_file_path = LaunchConfiguration('refine_engine_file_path')
    score_engine_file_path = LaunchConfiguration('score_engine_file_path')
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    image_input_topic = LaunchConfiguration('image_input_topic')
    camera_info_input_topic = LaunchConfiguration('camera_info_input_topic')
    depth_input_topic = LaunchConfiguration('depth_input_topic')
    image_height = LaunchConfiguration('image_height')
    image_width = LaunchConfiguration('image_width')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_bbox_to_mask = LaunchConfiguration('launch_bbox_to_mask')
    launch_tracking = LaunchConfiguration('launch_tracking')
    tf_frame = LaunchConfiguration('tf_frame')
    classes = LaunchConfiguration('classes')

    foundationpose_node = ComposableNode(
        name='foundationpose',
        package='isaac_ros_foundationpose',
        plugin='nvidia::isaac_ros::foundationpose::FoundationPoseNode',
        parameters=[{
            'mesh_file_path': mesh_file_path,
            'texture_path': texture_path,

            'refine_engine_file_path': refine_engine_file_path,
            'refine_input_tensor_names': ['input_tensor1', 'input_tensor2'],
            'refine_input_binding_names': ['input1', 'input2'],
            'refine_output_tensor_names': ['output_tensor1', 'output_tensor2'],
            'refine_output_binding_names': ['output1', 'output2'],

            'score_engine_file_path': score_engine_file_path,
            'score_input_tensor_names': ['input_tensor1', 'input_tensor2'],
            'score_input_binding_names': ['input1', 'input2'],
            'score_output_tensor_names': ['output_tensor'],
            'score_output_binding_names': ['output1'],

            'min_depth': 0.05,
            'tf_frame_name': tf_frame,
        }],
        remappings=[
            ('pose_estimation/depth_image', depth_topic),
            ('pose_estimation/image', image_topic),
            ('pose_estimation/camera_info', camera_info_topic),
            ('pose_estimation/segmentation', 'segmentation')]
    )

    selector_node = ComposableNode(
        name='selector_node',
        package='isaac_ros_foundationpose',
        plugin='nvidia::isaac_ros::foundationpose::Selector',
        parameters=[{
             # Expect to reset after the rosbag play complete
            'reset_period': 5000
        }],
        remappings=[
            ('image', image_topic),
            ('camera_info', camera_info_topic),
            ('depth_image', depth_topic),
        ]
        )

    foundationpose_tracking_node = ComposableNode(
        name='foundationpose_tracking_node',
        package='isaac_ros_foundationpose',
        plugin='nvidia::isaac_ros::foundationpose::FoundationPoseTrackingNode',
        parameters=[{
            'mesh_file_path': mesh_file_path,
            'texture_path': texture_path,

            'refine_engine_file_path': refine_engine_file_path,
            'refine_input_tensor_names': ['input_tensor1', 'input_tensor2'],
            'refine_input_binding_names': ['input1', 'input2'],
            'refine_output_tensor_names': ['output_tensor1', 'output_tensor2'],
            'refine_output_binding_names': ['output1', 'output2'],
        }],
    )

    detection2_d_to_mask_node = ComposableNode(
        name='detection2_d_to_mask',
        package='isaac_ros_foundationpose',
        plugin='nvidia::isaac_ros::foundationpose::Detection2DToMask',
        parameters=[{
            'mask_width': image_width,
            'mask_height': image_height,
        }],
        remappings=[
            ('detection2_d_array', '/detections_output')
        ]
        )

    foundationpose_tracking_container = ComposableNodeContainer(
        name='foundationpose_container',
        namespace='foundationpose_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[selector_node,
                                      foundationpose_tracking_node],
        output='screen',
        condition=IfCondition(launch_tracking)
    )

    foundationpose_bbox_container = ComposableNodeContainer(
        name='foundationpose_container',
        namespace='foundationpose_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            foundationpose_node,
            detection2_d_to_mask_node],
        output='screen',
        condition=IfCondition(launch_bbox_to_mask)
    )

    foundationpose_container = ComposableNodeContainer(
        name='foundationpose_container',
        namespace='foundationpose_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[foundationpose_node],
        output='screen',
        condition=UnlessCondition(launch_bbox_to_mask)
    )

    # my stuff

    detect_image_node = Node(
        package='geodude_perception',
        executable='detect_image',
        name='detect_image',
        parameters=[{
            'classes': classes,
        }],
        remappings=[
            ('image', image_topic),
        ]
    )

    camera_repub_node = Node(
        package='geodude_perception',
        executable='image_repub',
        name='camera_repub',
        remappings=[
            ('image_topic', image_input_topic),
            ('camera_info_topic', camera_info_input_topic),
            ('depth_topic', depth_input_topic),
            ('re_image_topic', image_topic),
            ('re_camera_info_topic', camera_info_topic),
            ('re_depth_topic', depth_topic),
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(launch_rviz)
    )

    return launch.LaunchDescription(launch_args + [foundationpose_container,
                                                   foundationpose_tracking_container,
                                                   foundationpose_bbox_container,
                                                   rviz_node, detect_image_node,
                                                   camera_repub_node])