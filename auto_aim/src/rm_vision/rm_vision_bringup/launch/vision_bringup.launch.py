import os
import sys
from ament_index_python.packages import get_package_share_directory

# 加入 common 模块路径
sys.path.append(os.path.join(
    get_package_share_directory('rm_vision_bringup'),
    'launch'
))

def generate_launch_description():
    from launch import LaunchDescription
    from launch.actions import Shutdown
    from launch_ros.actions import ComposableNodeContainer
    from launch_ros.descriptions import ComposableNode
    from common import node_params, launch_params, robot_state_publisher, tracker_node

    camera_node = ComposableNode(
        package='camera_core',
        plugin='camera::CameraCore',
        name='camera_core',
        parameters=[node_params],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # ArmorDetectorNode
    detector_node = ComposableNode(
        package='armor_detector',
        plugin='rm_auto_aim::ArmorDetectorNode',
        name='armor_detector',
        parameters=[node_params],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    serial_node = ComposableNode(
        package='rm_serial_driver',
        plugin='rm_serial_driver::RMSerialDriver',
        name='rm_serial_driver',
        parameters=[node_params],
    )

    container = ComposableNodeContainer(
        name='vision_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            camera_node,
            detector_node,
            serial_node,
            tracker_node,
        ],
        output='screen',
        emulate_tty=True,
        on_exit=Shutdown(),
    )

    return LaunchDescription([
        robot_state_publisher,
        container,
    ])
