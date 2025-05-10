from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_name = 'arj_simple_perception'
    pkg_dir = get_package_share_directory(pkg_name)


    return LaunchDescription([
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            arguments=['-d', [os.path.join(pkg_dir, 'config', 'rviz_config1.rviz')]]
        )
    ])