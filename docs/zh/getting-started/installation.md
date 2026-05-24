# 安装与环境配置

本页说明本地安装流程。**推荐优先使用 Docker 安装和运行**，因为容器环境已经处理好 ROS 2、Unitree ROS 2、CycloneDDS、ONNX Runtime、`libmotioncapture` 以及 mapping 依赖，能减少主机环境差异带来的问题。

Docker 安装见：[Docker 安装与运行](docker-installation.md)。

如果你需要在主机上直接开发、调试硬件接口或复用已有 ROS 2 环境，再按本文执行本地安装。

命令默认从 workspace 根目录执行，例如 `~/legged_ws`，而不是从仓库目录执行。

## 基础环境

推荐环境：

- Ubuntu 22.04 LTS
- ROS 2 Humble
- colcon
- rosdep
- Unitree ROS 2
- ONNX Runtime 1.22.0
- `libmotioncapture`

先安装 ROS 2 Humble desktop/full 或等价环境，并加载：

```bash
source /opt/ros/humble/setup.bash
```

## 安装系统依赖

安装基础依赖：

```bash
sudo apt update
sudo apt install -y --no-install-recommends \
  build-essential \
  ca-certificates \
  git \
  libyaml-cpp-dev \
  pkg-config \
  python3-colcon-common-extensions \
  python3-rosdep \
  wget \
  ros-humble-ament-cmake \
  ros-humble-imu-tools \
  ros-humble-xacro \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-rosidl-generator-dds-idl
```

初始化 rosdep：

```bash
sudo rosdep init || true
rosdep update
```

## 准备 Unitree ROS 2

Go2/G1 的底层通信依赖 Unitree ROS 2。建议将它放在独立目录，并构建其内部 `cyclonedds_ws`：

```bash
cd ~
git clone --branch master --depth 1 https://github.com/unitreerobotics/unitree_ros2.git
```

Unitree ROS 2 的 setup 脚本可能默认 source ROS 2 Foxy。当前仓库使用 Humble，需要改成 Humble：

```bash
sed -i 's|source /opt/ros/foxy/setup.bash|source /opt/ros/humble/setup.bash|g' ~/unitree_ros2/setup.sh
sed -i 's|source /opt/ros/foxy/setup.bash|source /opt/ros/humble/setup.bash|g' ~/unitree_ros2/setup_local.sh
```

构建 Unitree 内部 CycloneDDS workspace：

```bash
source ~/unitree_ros2/setup_local.sh
cd ~/unitree_ros2/cyclonedds_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

后续根据场景选择：

```bash
source ~/unitree_ros2/setup_local.sh
```

或者实机：

```bash
source ~/unitree_ros2/setup.sh
```

## 获取 legged_ros2

```bash
mkdir -p ~/legged_ws/src
cd ~/legged_ws/src
git clone https://github.com/zitongbai/legged_ros2.git
```

如果你已经在本地开发目录中，可以直接使用当前仓库。

## 准备 third_party 依赖

本地安装需要手动准备 `third_party` 依赖。

### ONNX Runtime

RL 控制器使用 ONNX Runtime。当前仓库期望目录：

```text
third_party/onnxruntime-linux-x64-1.22.0
```

安装：

```bash
cd ~/legged_ws/src/legged_ros2/third_party
wget https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-x64-1.22.0.tgz
tar -xzf onnxruntime-linux-x64-1.22.0.tgz
```

### libmotioncapture

`legged_mocap` 依赖 `third_party/libmotioncapture`：

```bash
cd ~/legged_ws/src/legged_ros2/third_party
git clone --branch main https://github.com/NOKOV-MOCAP/libmotioncapture.git
cd libmotioncapture
git submodule update --init --recursive
```

如果你不需要动捕功能，也建议保留该依赖，避免全仓库构建时缺少第三方目录。

## 构建 legged_ws

```bash
cd ~/legged_ws
source ~/unitree_ros2/setup_local.sh
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

> 实机环境下，构建通常仍可使用 `setup_local.sh`；运行实机节点时再切换为 `setup.sh`。

只构建部分包：

```bash
colcon build --symlink-install --packages-select legged_ros2_control
```

例如修改 RL 控制器和 Go2 配置后：

```bash
colcon build --symlink-install --packages-select legged_rl_controller go2_description
```

## 本地 mapping 依赖

如果你要在主机上直接运行 Go2 + MID360 + FAST-LIO，需要额外安装：

```bash
sudo apt update
sudo apt install -y --no-install-recommends cmake xterm
```

### Livox-SDK2

```bash
cd ~
git clone --branch master --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
cmake -S . -B build
cmake --build build -j"$(nproc)"
sudo cmake --install build
sudo ldconfig
```

### livox_ros_driver2

建议独立 workspace：

```bash
mkdir -p ~/livox_ws/src
cd ~/livox_ws/src
git clone --branch master --depth 1 https://github.com/Livox-SDK/livox_ros_driver2.git
source /opt/ros/humble/setup.bash
cd ~/livox_ws
rosdep install --from-paths src --ignore-src -r -y
cd ~/livox_ws/src/livox_ros_driver2
./build.sh humble
```

### FAST-LIO

同样建议独立 workspace，并在构建前 source Livox workspace：

```bash
mkdir -p ~/fast_lio_ws/src
cd ~/fast_lio_ws/src
git clone --branch ROS2 --depth 1 --recursive https://github.com/hku-mars/FAST_LIO.git
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
cd ~/fast_lio_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

本地 mapping 的启动流程见：[Go2 + MID360 + FAST-LIO 建图](../mapping/go2-mid360-fast-lio.md)。

## 运行环境脚本

本地运行时根据场景手动 source 对应环境。

仿真/本地：

```bash
source ~/unitree_ros2/setup_local.sh
source ~/legged_ws/install/setup.bash
export ROS_DOMAIN_ID=1
```

实机：

```bash
source ~/unitree_ros2/setup.sh
source ~/legged_ws/install/setup.bash
```

如果 CycloneDDS 需要绑定指定网卡，可以设置：

```bash
export NET_IF=eth0
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${NET_IF}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"
```

将 `eth0` 替换为连接机器人网络的网卡名。

## 验证安装

执行：

```bash
cd ~/legged_ws
source install/setup.bash
ros2 pkg list | grep legged
```

预期能看到：

- `legged_ros2_control`
- `legged_ros2_controller`
- `legged_rl_controller`
- `go2_description`
- `g1_description`
- `legged_mapping`
- `legged_mocap`

检查 Go2 broadcaster-only launch：

```bash
ros2 launch go2_description bringup_broadcasters.launch.py
```

如果只是验证模型、状态和 TF，优先使用 broadcaster-only 模式，因为它默认不向底层写低层命令。
