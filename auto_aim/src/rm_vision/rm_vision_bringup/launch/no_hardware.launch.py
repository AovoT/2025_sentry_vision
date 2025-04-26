import os, sys
from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import ComposableNodeContainer
from ament_index_python.packages import get_package_share_directory as gpsd
sys.path.append(os.path.join(gpsd('rm_vision_bringup'), 'launch'))

def generate_launch_description():

    from node_desc import rsp_component, detector_node, tracker_node

    container = ComposableNodeContainer(
        name='vision_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            detector_node,
            tracker_node,
            rsp_component,
        ],
        output='screen',
        emulate_tty=True,
        on_exit=Shutdown(),
    )
    return LaunchDescription([
        container
    ])