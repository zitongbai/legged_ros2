# Go2 快速启动

本页给出 Go2 的最短运行路径。它假设你已经完成[安装与环境配置](installation.md)，并且已经构建 `~/legged_ws`。

## 启动前检查

确认：

- 已经 source Unitree ROS 2 setup。
- 已经 source `~/legged_ws/install/setup.bash`。
- ONNX Runtime 已放在 `third_party/onnxruntime-linux-x64-1.22.0`。
- Go2 的 RL policy 和 `IO_descriptors.yaml` 位于 `go2_description/config/rl_policy/`。

仿真时通常还需要：

```bash
export ROS_DOMAIN_ID=1
```

因为 Unitree Mujoco 默认使用 `ROS_DOMAIN_ID=1`。

## 只启动状态广播

如果你只是想检查模型、关节状态、IMU 和 TF，先运行：

```bash
source path/to/unitree_ros2/setup_local.sh
source ~/legged_ws/install/setup.bash
export ROS_DOMAIN_ID=1
ros2 launch go2_description bringup_broadcasters.launch.py
```

这个 launch 默认：

- 启动 `go2_main_loop`。
- 启动 `robot_state_publisher`。
- 加载 `joint_state_broadcaster` 和 `imu_state_broadcaster`。
- `enable_lowlevel_write` 默认为 `false`。
- `use_rviz` 默认为 `true`。

## 启动 RL bringup

仿真中：

```bash
source path/to/unitree_ros2/setup_local.sh
source ~/legged_ws/install/setup.bash
export ROS_DOMAIN_ID=1
ros2 launch go2_description bringup_rl.launch.py use_rviz:=true use_rqt_cm:=true
```

实机中：

```bash
source path/to/unitree_ros2/setup.sh
source ~/legged_ws/install/setup.bash
ros2 launch go2_description bringup_rl.launch.py use_rviz:=true use_rqt_cm:=true
```

`bringup_rl.launch.py` 会加载：

- `joint_state_broadcaster`
- `imu_state_broadcaster`
- `stand_static_controller`
- `sit_static_controller`
- `rl_controller`

其中 `rl_controller`、站立和坐下控制器会以 inactive 状态加载，等待手柄或 controller manager 切换。

## 手柄控制

Go2 默认 main loop 配置位于：

```text
go2_description/config/main_loop/rl.yaml
```

默认按键绑定：

```text
LB + A  -> stand_static_controller
LB + B  -> sit_static_controller
LB + X  -> rl_controller
LB + RB -> off
```

速度指令由手柄摇杆转换为 `/cmd_vel`，缩放参数也在 main loop YAML 中配置。

## 常用检查命令

查看 controller 状态：

```bash
ros2 control list_controllers
```

查看硬件接口：

```bash
ros2 control list_hardware_interfaces
```

查看关节状态：

```bash
ros2 topic echo /joint_states
```

查看 IMU：

```bash
ros2 topic echo /imu
```

## 最小安全建议

第一次运行实机时，先用：

```bash
ros2 launch go2_description bringup_broadcasters.launch.py
```

确认状态和 TF 正常，再切换到 static controller，最后再尝试 RL controller。不要在空间狭小或机器人未受控支撑时直接启动 RL。
