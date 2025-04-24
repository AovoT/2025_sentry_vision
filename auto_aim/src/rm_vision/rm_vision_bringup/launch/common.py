import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

# 加载 launch_params
params_path = os.path.join(
    get_package_share_directory('rm_vision_bringup'),
    'config', 'launch_params.yaml'
)
launch_params = yaml.safe_load(open(params_path))

# 构造 xacro 路径
joint = 'gimbal_link_to_camera_link'
xacro_path = os.path.join(
    get_package_share_directory('sentry'),
    'urdf', 'rm_gimbal.urdf.xacro'
)
assert os.path.exists(xacro_path), f"Xacro 文件不存在: {xacro_path}"

# 调试打印
print("Xacro command:", [
    'xacro ' + xacro_path + 
    ' xyz:=' + launch_params[joint]['xyz'] +
    ' rpy:=' + launch_params[joint]['rpy']
])

# 3. 拼接 xacro 命令
robot_description = Command([
    'xacro ' +
    xacro_path +
    ' xyz:=' + launch_params[joint]['xyz'] +   # e.g. "0.07 -0.012 -0.011"
    ' rpy:=' + launch_params[joint]['rpy']   # e.g. "0.0 0.0 0.0"
])
# 4. robot_state_publisher 节点，接收动态生成的 URDF
robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{
        'robot_description': robot_description,
        'publish_frequency': 1000.0,
    }],
)

# 5. Tracker 参数与 ComposableNode
node_params = os.path.join(
    get_package_share_directory('rm_vision_bringup'),
    'config', 'node_params.yaml'
)
assert os.path.exists(node_params), f"Tracker 参数文件不存在: {node_params}"

tracker_node = ComposableNode(
    package='armor_tracker',
    plugin='rm_auto_aim::ArmorTrackerNode',
    name='armor_tracker',
    parameters=[node_params],
)
