# 河北科大 AT 视觉系统

本项目为基于 ROS2 的双PTZ视觉系统，支持装甲板识别等视觉任务，适用于 RoboMaster 。

## 目录结构

- `src/hikcamera/`：海康相机驱动模块
- `src/rm_auto_aim/`：装甲板自动瞄准算法模块
- `src/rm_gimbal_description/`：机器人云台 URDF 描述
- `src/rm_serial_driver/`：串口通讯模块
- `src/rm_vision/`：视觉算法集成
- `docs/`：开发与使用文档

## 环境要求

- Ubuntu 22.04
- ROS2 Humble
- OpenCV 4.5+

## 快速开始

### 1. 安装依赖

```sh
rosdep install --from-paths src --ignore-src -r -y
sudo apt install ros-humble-serial-driver
```

### 2. 克隆仓库

```sh
git clone https://gitee.com/SMBU-POLARBEAR/PB_RM_Vision
cd PB_RM_Vision
```

### 3. 编译项目

```sh
cd script
./buildit.sh
```

### 4. 环境配置

```sh
source install/setup.bash
```

## 启动方法

### 启动所有模块（哨兵）

```sh
sudo chmod +x script/use_udev_rules.sh
source install/setup.bash
ros2 launch rm_vision_bringup vision_bringup.launch.py
```

视觉集成 | `ros2 launch rm_vision_bringup vision_bringup.launch.py`

### 单独运行子模块

- 装甲板识别模块 && 海康相机模块

  ```sh
  ros2 launch detector detector.launch.py
  ```

- 串口模块

  ```sh
  ros2 launch rm_serial_driver serial_driver.launch.py
  ```

## 可视化

```sh
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

## 相关文档

- [ROS2的使用](docs/ROS2的使用.md)
- [进度与规划](docs/进度与规划.md)
- [rm_vision 部署文档](https://flowus.cn/lihanchen/share/0d472992-f136-4e0e-856f-89328e99c684)
- [相机标定](https://flowus.cn/lihanchen/share/02a518a0-f1bb-47a5-8313-55f75bab21b5)

---

如需详细开发说明、接口文档或遇到问题，请查阅 [docs/](docs/) 目录下相关文档

## 鸣谢

 [北极熊视觉系统](https://gitee.com/SMBU-POLARBEAR/PB_RM_Vision)

 [rm_vision](https://github.com/rm-vision-archive/rm_vision)

 [RMCS](https://github.com/Alliance-Algorithm/RMCS)
