import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  # 获取参数文件路径
  config_file = os.path.join(
    get_package_share_directory('rm_serial_driver'),
    'config',
    'rm_serial_driver.yaml'
  )

  # 定义节点信息
  rm_serial_driver_node = launch_ros.actions.Node(
    package='rm_serial_driver',
    executable='rm_serial_driver_node',
    name='rm_serial_driver',
    output='screen',
    parameters=[config_file]
  )
  return launch.LaunchDescription([
    rm_serial_driver_node
  ])