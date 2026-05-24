# 常见问题

本页整理运行 `legged_ros2` 时常见的错误和排查方向。

## 找不到包或 launch

现象：

```text
Package '<name>' not found
```

检查：

```bash
source /opt/ros/humble/setup.bash
source ~/legged_ws/install/setup.bash
ros2 pkg list | grep legged
```

如果包不存在，重新构建：

```bash
cd ~/legged_ws
colcon build --symlink-install
source install/setup.bash
```

## 仿真中收不到 Unitree 消息

优先检查 `ROS_DOMAIN_ID`：

```bash
echo $ROS_DOMAIN_ID
```

Unitree Mujoco 默认通常是：

```bash
export ROS_DOMAIN_ID=1
```

还要确认启动控制节点前 source 了：

```bash
source path/to/unitree_ros2/setup_local.sh
```

## 实机中收不到机器人消息

检查：

- PC 和机器人是否在同一网络。
- PC 网卡 IP 是否按 Unitree 文档配置。
- 是否 source 了 `unitree_ros2/setup.sh`。
- Docker 中是否设置了 `NET_IF`。
- CycloneDDS 是否绑定到了正确网卡。

Docker 示例：

```bash
export NET_IF=eth0
source /root/legged_ws/setup.sh
```

## controller configure 失败

查看 controller：

```bash
ros2 control list_controllers
```

查看硬件接口：

```bash
ros2 control list_hardware_interfaces
```

常见原因：

- controller YAML 中 joint 名称不存在。
- URDF `ros2_control` 中没有对应 command/state interface。
- IMU 名称不匹配。
- pluginlib 注册名错误。
- 硬件接口初始化失败。

## RL policy 文件找不到

检查路径：

```text
go2_description/config/rl_policy/policy.onnx
go2_description/config/rl_policy/IO_descriptors.yaml
```

Go2 默认 controller 配置使用 package URL：

```yaml
onnx_model_path: "package://go2_description/config/rl_policy/policy.onnx"
io_descriptors_path: "package://go2_description/config/rl_policy/IO_descriptors.yaml"
```

如果使用自定义路径，确认 launch 或 YAML 中传入了正确路径。

## Observation term 未注册

现象：

```text
Observation term '<name>' is not registered.
```

说明 `IO_descriptors.yaml` 中包含某个 observation term，但 C++ 运行时没有对应实现或注册。

处理：

- 确认 policy 和 descriptor 是否来自当前代码支持的导出。
- 如果新增了 observation，在 C++ 中实现并注册。
- 不要只替换 descriptor 而不更新代码。

## ONNX 输入名不匹配

现象：

```text
Input name <...> not found in observations.
```

说明 ONNX 模型期望的输入名和 observation manager 生成的 group 名称不一致。

处理：

- 使用同一次导出的 ONNX 和 descriptor。
- 检查 observation group。
- 如果只有一个 observation group，当前运行时可能会映射为 `obs`，需要和导出保持一致。

## Action size mismatch

现象：

```text
Action size mismatch
```

常见原因：

- policy 输出维度和受控关节数量不一致。
- `rl_controller.joint_names` 配置错了。
- 使用了错误机器人对应的 policy。
- action term 语义改变但 C++ 未同步修改。

## 机器人动作异常但没有报错

优先怀疑顺序问题：

- `rl_controller.joint_names`
- `IO_descriptors.yaml` 中 `articulations.robot.joint_names`
- policy action 输出顺序
- 硬件接口中 joint 到 motor index 的映射

这些顺序只要有一个不一致，程序可能仍能运行，但机器人动作会错误。

## RViz 中 TF 不完整

检查：

```bash
ros2 run tf2_tools view_frames
```

常见原因：

- `robot_state_publisher` 未启动。
- joint state broadcaster 未启动。
- description 中 link/joint 名称和配置不一致。
- mapping 场景中 FAST-LIO 的 `camera_init -> body` 未发布。
- `ground_odom_tf_node` 启动早于脚部 TF 可用。

## ground_odom_tf_node 退出

常见原因：

- 配置的 foot frame 不存在。
- 启动时 joint state/robot state publisher 还没准备好。
- 脚点少于 3 个。
- 参数中的旋转顺序或角度单位非法。

推荐启动顺序：

```text
Livox driver
FAST-LIO
bringup_broadcasters
ground_odom_tf_node
```

## 实机运行前的最低安全清单

- 先运行 broadcaster-only。
- 确认 joint state 和 IMU 正常。
- 确认 controller list 正常。
- 确认 Unitree 原生运动服务不会同时控制电机。
- 先激活 static controller。
- 再以小速度测试 RL。
- 随时准备停止 controller 或急停。
