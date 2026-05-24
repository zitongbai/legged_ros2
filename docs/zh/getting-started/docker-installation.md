# Docker 安装与运行

推荐优先使用 Docker 运行 `legged_ros2`。仓库中的 Dockerfile 已经固定了基础镜像、ROS 2 Humble、Unitree ROS 2、CycloneDDS workspace、ROS 2 control、ONNX Runtime 和动捕依赖的安装方式；mapping 镜像还包含 Livox-SDK2、livox_ros_driver2 和 FAST-LIO。

本页命令从仓库根目录执行：

```bash
cd ~/legged_ws/src/legged_ros2
```

## 普通控制镜像

构建镜像：

```bash
docker/build.sh
```

默认镜像名：

```text
legged-ros2:humble
```

可以通过环境变量覆盖：

```bash
IMAGE_NAME=my-legged-ros2:humble docker/build.sh
```

`docker/build.sh` 会使用 `docker/Dockerfile`，主要完成：

- 使用 `osrf/ros:humble-desktop-full`。
- 安装构建工具、`rosdep`、`colcon`、`xacro`、`ros2_control`、`ros2_controllers`、CycloneDDS RMW 等依赖。
- clone `unitree_ros2`。
- 将 Unitree setup 脚本中的 Foxy source 修改为 Humble source。
- source `unitree_ros2/setup_local.sh`。
- 在 `/root/unitree_ros2/cyclonedds_ws` 中安装依赖并 `colcon build --symlink-install`。
- 复制容器内的 `/root/legged_ws/setup.sh` 和 `/root/legged_ws/setup_local.sh`。

## 启动普通容器

```bash
docker/run.sh
```

默认容器名：

```text
legged-ros2-humble
```

`docker/run.sh` 会先在主机侧准备：

- `third_party/onnxruntime-linux-x64-1.22.0`
- `third_party/libmotioncapture`

然后启动容器并挂载：

```text
<repo> -> /root/legged_ws/src/legged_ros2
```

容器使用 host network。若主机设置了 `DISPLAY`，脚本会挂载 X11 socket 和 `/dev/dri`，用于 RViz/rqt 等 GUI 程序。

容器启动后，脚本会在容器内执行：

```bash
source /root/unitree_ros2/setup_local.sh
cd /root/legged_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

完成后自动进入交互 shell。

## 进入已有普通容器

```bash
docker/enter.sh
```

它只进入已经运行的 `legged-ros2-humble` 容器，不会重新创建或初始化 workspace。

## 容器内环境脚本

仿真/本地通信：

```bash
source /root/legged_ws/setup_local.sh
```

这个脚本会：

- source `/root/unitree_ros2/setup_local.sh`
- source `/root/legged_ws/install/setup.bash`
- 设置 `ROS_DOMAIN_ID=1`

实机通信：

```bash
source /root/legged_ws/setup.sh
```

这个脚本会：

- source `/root/unitree_ros2/setup.sh`
- 自动从默认路由检测网卡，或使用 `NET_IF`
- 重新设置 `CYCLONEDDS_URI`
- source `/root/legged_ws/install/setup.bash`

如果需要指定网卡：

```bash
export NET_IF=eth0
source /root/legged_ws/setup.sh
```

## 普通容器中运行 Go2

仿真：

```bash
source /root/legged_ws/setup_local.sh
ros2 launch go2_description bringup_rl.launch.py use_rviz:=true use_rqt_cm:=true
```

实机：

```bash
export NET_IF=<robot-network-interface>
source /root/legged_ws/setup.sh
ros2 launch go2_description bringup_rl.launch.py use_rviz:=true use_rqt_cm:=true
```

只检查状态和 TF：

```bash
source /root/legged_ws/setup_local.sh
ros2 launch go2_description bringup_broadcasters.launch.py
```

## Mapping 镜像

如果要运行 Go2 + MID360 + FAST-LIO，使用 mapping 镜像：

```bash
docker/build_mapping.sh
```

默认镜像名：

```text
legged-ros2-mapping:humble
```

`docker/build_mapping.sh` 使用 `docker/Dockerfile.mapping`。它在普通控制镜像依赖基础上额外完成：

- 安装 `cmake` 和 `xterm`。
- clone、构建并安装 Livox-SDK2 到 `/usr/local`。
- 在 `/root/livox_ws` 中 clone 并构建 `livox_ros_driver2`。
- 在 `/root/fast_lio_ws` 中 clone 并构建 FAST-LIO 的 ROS 2 分支。

## 启动 Mapping 容器

```bash
docker/run_mapping.sh
```

默认容器名：

```text
legged-ros2-mapping-humble
```

除了挂载仓库，它还会挂载 MID360 配置：

```text
legged_mapping/config/MID360_config.json
  -> /root/livox_ws/src/livox_ros_driver2/config/MID360_config.json
```

启动后会在容器内：

```bash
source /root/unitree_ros2/setup_local.sh
cd /root/legged_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

source /opt/ros/humble/setup.bash
cd /root/livox_ws/src/livox_ros_driver2
./build.sh humble
```

重新执行 `./build.sh humble` 是为了确保 Livox driver 使用挂载进来的 MID360 配置。

进入已有 mapping 容器：

```bash
docker/enter_mapping.sh
```

## Mapping 容器中运行

先指定连接机器人/MID360 网络的网卡：

```bash
export NET_IF=<your-local-network-interface>
```

例如：

```bash
export NET_IF=enp3s0
```

可以按建图文档手动开四个终端，也可以使用脚本：

```bash
export NET_IF=<your-local-network-interface>
bash /root/legged_ws/src/legged_ros2/scripts/run_mapping_terminals.sh
```

详细流程见：[Go2 + MID360 + FAST-LIO 建图](../mapping/go2-mid360-fast-lio.md)。

## 常用环境变量

- `IMAGE_NAME`：覆盖镜像名。
- `CONTAINER_NAME`：覆盖容器名。
- `ONNX_VERSION`：覆盖 ONNX Runtime 版本，默认 `1.22.0`。
- `LIBMOTIONCAPTURE_REPO`：覆盖 `libmotioncapture` 仓库地址。
- `LIBMOTIONCAPTURE_REF`：覆盖 `libmotioncapture` 分支。
- `NET_IF`：实机或 mapping 网络接口。
- `HTTP_PROXY` / `HTTPS_PROXY` / `ALL_PROXY`：构建镜像时透传代理。

## 清理和重建

`docker/run.sh` 和 `docker/run_mapping.sh` 如果发现同名容器已存在，会先删除旧容器再创建新的容器。

如果只是进入已有容器，使用：

```bash
docker/enter.sh
```

或者：

```bash
docker/enter_mapping.sh
```
