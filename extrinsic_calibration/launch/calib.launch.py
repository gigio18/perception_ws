import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("extrinsic_calibration"),
        'config',
        'calibration.yaml'
    )

    return LaunchDescription([
        Node(
            package ="extrinsic_calibration",
            executable = "extrinsic",
            name='extrinsic_calib_node',
            output = "screen",
            emulate_tty = True,
            parameters=[config_file]
        )
    ])