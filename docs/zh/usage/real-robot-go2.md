# Go2 实机运行

本页说明如何在真实 Unitree Go2 上运行 `legged_ros2`。实机运行有风险，第一次调试应有人在旁协助，并确保机器人周围有足够空旷区域。

## 安全检查

运行前确认：

- 机器人放在平坦、稳定、开阔的区域。
- 操作者熟悉急停和断电流程。
- 电脑和机器人网络稳定。
- 不要在不确定 controller 状态时直接启动 RL。
- 第一次调试优先使用 `bringup_broadcasters.launch.py`，确认状态通路正常。

## 网络连接

将 PC 通过以太网连接到 Go2。PC 网卡 IP、子网和 Unitree 网络配置需要按 Unitree 官方说明设置。

如果在 Docker 中运行，需要设置 CycloneDDS 使用的网卡：

```bash
export NET_IF=<your-network-interface>
source /root/legged_ws/setup.sh
```

例如：

```bash
export NET_IF=eth0
source /root/legged_ws/setup.sh
```

## 停止 Go2 原生运动服务

运行低层控制前，需要确认 Go2 自带运动控制服务不会同时控制电机。可以使用 Unitree SDK2 中的示例程序停止原生服务。

示例：

```bash
cd path/to/unitree_sdk2/build/bin
./go2_stand_example
```

运行这个命令前，确认机器人处于安全姿态。完成后关闭该终端，再启动本仓库控制栈。

## 启动状态广播

先只启动 broadcaster：

```bash
source path/to/unitree_ros2/setup.sh
source ~/legged_ws/install/setup.bash
ros2 launch go2_description bringup_broadcasters.launch.py
```

这个模式默认 `enable_lowlevel_write:=false`，适合检查 joint state、IMU 和 TF。

检查：

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states
```

确认状态正常后，再进入可写控制模式。

## 启动 RL bringup

```bash
source path/to/unitree_ros2/setup.sh
source ~/legged_ws/install/setup.bash
ros2 launch go2_description bringup_rl.launch.py use_rviz:=true use_rqt_cm:=true
```

默认 `enable_lowlevel_write:=true`，会允许底层命令写入机器人。

## 默认手柄绑定

```text
LB + A  -> stand_static_controller
LB + B  -> sit_static_controller
LB + X  -> rl_controller
LB + RB -> off
```

旧文档中也可能写作 `L1`，对应同一个左肩键概念。

## 推荐实机流程

1. 启动 broadcaster-only，确认状态和 TF。
2. 启动 RL bringup，但先不要切 RL。
3. 使用 `LB + A` 激活站立静态控制器。
4. 确认机器人站立稳定。
5. 使用较小速度指令激活 RL。
6. 随时准备 `LB + RB` 停止控制器或执行急停。

## 不建议的操作

- 不确认 joint order 就替换 policy。
- ONNX 和 `IO_descriptors.yaml` 来自不同导出版本。
- 在机器人悬空、被卡住或附近有人时启动 RL。
- 同时运行 Unitree 原生运动服务和本仓库低层控制。
