import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    def load_urdf_file(file_path: str) -> str:
        with open(file_path, 'r') as f:
            return f.read()

    bringup_dir = get_package_share_directory('sentry')
    default_urdf_path = os.path.join(bringup_dir, 'urdf', 'rm_gimbal.urdf')

    rviz_config_file = os.path.join(bringup_dir, 'rviz', 'view.rviz')

    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=default_urdf_path,
        description='URDF file path'
    )
    use_robot_state_pub_arg = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start robot_state_publisher'
    )
    use_joint_state_pub_arg = DeclareLaunchArgument(
        'use_joint_state_pub',
        default_value='True',
        description='Whether to start joint_state_publisher'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ'
    )

    urdf_file = LaunchConfiguration('urdf_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_joint_state_pub = LaunchConfiguration('use_joint_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')

    robot_description_content = load_urdf_file(default_urdf_path)

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    start_joint_state_publisher_cmd = Node(
        condition=IfCondition(use_joint_state_pub),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )

    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(urdf_file_arg)
    ld.add_action(use_robot_state_pub_arg)
    ld.add_action(use_joint_state_pub_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    return ld

