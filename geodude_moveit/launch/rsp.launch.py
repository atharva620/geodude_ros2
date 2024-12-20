from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch
from launch import LaunchDescription


def generate_launch_description():
    """ RSP should be launched by geodude_hardware
    moveit_config = MoveItConfigsBuilder("geodude", package_name="geodude_moveit").to_moveit_configs()
    return generate_rsp_launch(moveit_config)
    """
    return LaunchDescription()
