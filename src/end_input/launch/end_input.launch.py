import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    end_input_node = Node(
        package='end_input',
        executable='end_input_node',
        namespace='',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([end_input_node])
