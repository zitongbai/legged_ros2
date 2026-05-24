# RL 控制定制

`legged_rl_controller` 负责在 ROS 2 控制循环中运行 RL policy。它读取状态，构造 observation，调用 ONNX Runtime 推理，再把 action 转换成关节命令。

## 相关文件

主要代码：

```text
legged_rl_controller/src/legged_rl_controller.cpp
legged_rl_controller/include/legged_rl_controller/legged_rl_controller.hpp
legged_rl_controller/include/legged_rl_controller/legged_articulation.hpp
```

Go2 默认 policy 资源：

```text
legged_robot_description/go2_description/config/rl_policy/policy.onnx
legged_robot_description/go2_description/config/rl_policy/IO_descriptors.yaml
```

Go2 controller 配置：

```text
legged_robot_description/go2_description/config/ros2_control/rl.yaml
```

## ONNX 和 IO descriptors

`policy.onnx` 是策略网络。`IO_descriptors.yaml` 描述策略输入输出、observation term、action term、关节顺序等信息。

这两个文件必须来自同一次导出。不要只替换 ONNX，不替换 descriptor；也不要手工修改 descriptor 来适配一个不匹配的 policy。

## 必须保持一致的内容

以下内容必须一致：

- ONNX 输入名和 observation group 名称。
- `IO_descriptors.yaml` 中 observation term 名称和 C++ 注册名称。
- `IO_descriptors.yaml` 中 action term 名称和 C++ 注册名称。
- action 输出维度和受控关节数量。
- `rl_controller.joint_names` 顺序和 policy/export 中的关节顺序。
- controller 配置中的 IMU 名称和 URDF `ros2_control` sensor 名称。

最容易出错的是 joint order。即使 joint 名称集合相同，顺序不一致也会导致策略输出施加到错误关节上。

## 只替换策略权重

如果 observation/action schema 没变：

1. 用同一次导出的 `policy.onnx` 替换旧文件。
2. 用同一次导出的 `IO_descriptors.yaml` 替换旧文件。
3. 不需要修改 C++。
4. 重新构建 description 包或确认 symlink install 已生效。

示例：

```bash
colcon build --symlink-install --packages-select go2_description
```

## 修改 observation

如果新增或修改 observation：

1. 在训练/导出侧更新 Isaac Lab observation 配置。
2. 重新导出 `policy.onnx` 和 `IO_descriptors.yaml`。
3. 在 C++ 中实现对应 observation term。
4. 确保 term 名称和 descriptor 完全一致。
5. 如果需要新的机器人状态，更新 `LeggedArticulation` 的状态更新路径。

常见错误：

```text
Observation term '<name>' is not registered.
```

说明 descriptor 中出现了运行时未注册的 observation term。

## 修改 action

当前常见 action 是关节位置动作。修改 action 语义时，需要同时考虑：

- Isaac Lab 导出配置。
- descriptor 中的 action term 名称。
- C++ action term 实现。
- action 维度。
- 关节名称和顺序。

常见错误：

```text
Action size mismatch
```

通常说明 policy 输出维度和 `rl_controller.joint_names` 数量不一致。

## 命令速度输入

RL controller 订阅 `cmd_vel`。Go2 默认配置：

```yaml
cmd_vel_topic: "cmd_vel"
cmd_vel_range_lin_vel_x: [-1.0, 1.0]
cmd_vel_range_lin_vel_y: [-1.0, 1.0]
cmd_vel_range_ang_vel_z: [-3.14, 3.14]
```

手柄输入到 `cmd_vel` 的缩放在 main loop 配置中：

```text
go2_description/config/main_loop/rl.yaml
```

## 验证清单

修改 RL 后至少检查：

```bash
colcon build --symlink-install --packages-select legged_rl_controller go2_description
ros2 launch go2_description bringup_rl.launch.py use_rqt_cm:=true
ros2 control list_controllers
```

确认没有以下错误：

- policy 文件找不到。
- descriptor 文件找不到。
- ONNX input name 不匹配。
- observation term 未注册。
- action size mismatch。
- IMU interface 不存在。

实机验证时先激活 static controller，再以低速度、小幅度测试 RL。

## Sim2Sim / Sim2Real 注意事项

策略从仿真迁移到仿真或实机时，重点检查：

- 关节顺序。
- MuJoCo joint `armature`。
- IMU 安装位置和方向。
- 关节限位。
- 默认关节角。
- PD 增益和底层电机模式。

RL policy 可能利用训练环境中的 joint limit 或动力学细节。如果实机和训练环境不一致，策略可能表现不稳定。
