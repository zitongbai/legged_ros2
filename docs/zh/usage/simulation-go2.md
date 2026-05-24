# Go2 仿真运行

本页说明如何配合 Unitree Mujoco 运行 Go2 仿真，并启动 `legged_ros2` 的控制栈。

## 前置条件

需要先安装并构建 Unitree Mujoco。建议使用 Unitree Mujoco 的 C++ simulator，性能更稳定。

在 Unitree Mujoco 的配置中确认：

```yaml
use_joystick: 1
```

Unitree Mujoco 默认使用：

```bash
export ROS_DOMAIN_ID=1
```

启动任何和仿真通信的 ROS 2 节点前，都应确认终端中的 `ROS_DOMAIN_ID` 一致。

## 启动仿真器

在 Unitree Mujoco 的 build 目录中运行：

```bash
cd path/to/unitree_mujoco/simulate/build
./unitree_mujoco
```

如果仿真器启动时出现 DDS 或环境冲突，可以尝试在一个没有 source ROS 2 setup 的新终端中启动仿真器。

## 启动 Go2 控制栈

打开新终端：

```bash
source path/to/unitree_ros2/setup_local.sh
source ~/legged_ws/install/setup.bash
export ROS_DOMAIN_ID=1
ros2 launch go2_description bringup_rl.launch.py use_rviz:=true use_rqt_cm:=true
```

这会启动：

- Go2 `ros2_control` main loop。
- `robot_state_publisher`。
- joint/IMU broadcaster。
- stand/sit static controller。
- RL controller。
- 可选 RViz 和 rqt controller manager。

## 控制器切换

默认按键：

```text
LB + A  -> 站立控制器
LB + B  -> 坐下控制器
LB + X  -> RL 控制器
LB + RB -> 停止所有控制器
```

速度指令会发布到 `cmd_vel`，RL controller 根据 `cmd_vel` 生成相应运动。

## 调试建议

先检查 controller：

```bash
ros2 control list_controllers
```

再检查硬件接口：

```bash
ros2 control list_hardware_interfaces
```

如果 controller 无法 configure，优先检查：

- `go2_description/config/ros2_control/rl.yaml` 中 joint 和 IMU 名称。
- URDF/Xacro 中的 `ros2_control` joint 和 sensor。
- Unitree Mujoco 是否已经启动。
- `ROS_DOMAIN_ID` 是否一致。

## 常见问题

仿真器和控制节点互相收不到消息时，通常是 DDS 环境或 `ROS_DOMAIN_ID` 不一致。

RL controller 启动后报 ONNX 或 descriptor 错误时，检查：

```text
go2_description/config/rl_policy/policy.onnx
go2_description/config/rl_policy/IO_descriptors.yaml
```

两者必须来自同一次策略导出。
