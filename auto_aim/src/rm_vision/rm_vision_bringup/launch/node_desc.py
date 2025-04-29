import os
import yaml

from launch_ros.descriptions import ComposableNode
from launch.substitutions import Command

from ament_index_python.packages import get_package_share_directory as gpsd


# 加载 launch_params
launch_params_path = os.path.join(
    gpsd('rm_vision_bringup'),
    'config', 'launch_params.yaml'
)

node_params = os.path.join(
    gpsd('rm_vision_bringup'),
    'config', 'node_params.yaml'
)

cfg = yaml.safe_load(open(launch_params_path, 'r'))

xyz = cfg['gimbal_link_to_camera_link']['xyz']    # "0.07 -0.012 -0.011"
rpy = cfg['gimbal_link_to_camera_link']['rpy']    # "0 0 0"

# ---------- 2. xacro → robot_description ----------
xacro_file = os.path.join(
    gpsd('sentry'), 'urdf', 'rm_gimbal.urdf.xacro')

robot_description = Command([
    'xacro ' + xacro_file +
    ' xyz:=' + xyz +
    ' rpy:=' + rpy
])

rsp_component = ComposableNode(
    package='robot_state_publisher',
    plugin='robot_state_publisher::RobotStatePublisher',
    name='robot_state_publisher',
    parameters=[{
        'robot_description': robot_description,
        'publish_frequency': 1000.0
    }]
)


# ArmorDetectorNode
detector_node = ComposableNode(
    package='armor_detector',
    plugin='rm_auto_aim::ArmorDetectorNode',
    name='armor_detector',
    parameters=[node_params],
    extra_arguments=[{'use_intra_process_comms': True}],
)

tracker_node = ComposableNode(
    package='armor_tracker',
    plugin='rm_auto_aim::ArmorTrackerNode',
    name='armor_tracker',
    parameters=[node_params],
    extra_arguments=[{'use_intra_process_comms': True}],
)

serial_node = ComposableNode(
    package='rm_serial_driver',
    plugin='rm_serial_driver::RMSerialDriver',
    name='rm_serial_driver',
    parameters=[node_params],
    extra_arguments=[{'use_intra_process_comms': True}],
)