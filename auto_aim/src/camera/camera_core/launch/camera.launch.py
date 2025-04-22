from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

node_params = os.path.join(
    get_package_share_directory('camera_core'), 'config', 'config.yaml')

camera_node = Node(
    package='camera_core',
    executable='camera_core_node',
    name='camera_core',  # 若YAML中有/camera_core则需要加
    output='screen',
    emulate_tty=True,
    parameters=[node_params],
    arguments=['--ros-args', '--log-level', 'info'],
)


def generate_launch_description():
    return LaunchDescription([
        camera_node
    ])
