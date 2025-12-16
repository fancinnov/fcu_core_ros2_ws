## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS2**
Ubuntu 64-bit 22.04.
ROS Humble. [ROS2 Installation](https://docs.ros.org/en/humble/)

### 1.2. **serial**
sudo apt install ros-humble-serial-driver

### 1.3. **eigen**
sudo apt-get install libeigen3-dev

## 在fcu_core_ros2_ws中打开终端
## 2. Build
colcon build

## 3. Source
source install/setup.bash

## 4. 如果用到串口需要配置权限
sudo chmod 777 /dev/ttyACM0

## 5. 运行launch文件
ros2 launch fcu_core fcu_core_launch.py