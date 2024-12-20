from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("geodude", package_name="geodude_moveit").to_moveit_configs()
    ld = generate_demo_launch(moveit_config)

    # Pop 2nd to last (ros2 controller)
    # As those are handled by geodude_hardware on a different machine
    ld.entities.pop(-2)
    return ld

