# 系统架构

`legged_ros2` 的核心设计是分层和插件化。机器人底层通信、`ros2_control` 硬件接口、控制器、机器人描述和任务工具相互解耦，通过 ROS 2 topic、TF、controller interface 和 YAML 配置连接。

## 总体分层

```text
应用/任务层
  navigation, mapping, mocap, teleop

控制策略层
  legged_ros2_controller
  legged_rl_controller

ros2_control 运行层
  controller_manager
  LeggedRos2Control main loop

硬件抽象层
  LeggedSystemInterface
  Go2SystemInterface
  G1SystemInterface

机器人/仿真后端
  Unitree SDK2 / Unitree ROS 2 / Unitree Mujoco
```

## 运行时主循环

Go2/G1 的 main loop 在 `legged_ros2_control` 中实现，典型流程是：

```text
hardware read()
controller_manager update()
hardware write()
```

`read()` 从底层消息读取关节、IMU 等状态；controller 根据状态产生命令；`write()` 将命令写入底层 lowcmd 并发布。

## 硬件接口

硬件接口基类是：

```text
legged_ros2_control/include/legged_ros2_control/legged_system_interface.hpp
```

当前注册的硬件插件在：

```text
legged_ros2_control/legged_ros2_control_plugins.xml
```

包括：

- `legged_ros2_control/Go2SystemInterface`
- `legged_ros2_control/G1SystemInterface`

机器人描述中的 `ros2_control` Xacro 会选择具体插件。插件名、URDF 中的 joint/sensor、controller YAML 中的 joint/imu 名称必须一致。

## 控制器插件

经典控制器注册在：

```text
legged_ros2_controller/legged_ros2_controller_plugins.xml
```

主要包括：

- `legged_ros2_controller/StaticController`
- `legged_ros2_controller/RandomTrajController`
- `legged_ros2_controller/SeparateJointController`
- `legged_ros2_controller/LeggedController`

RL 控制器注册在：

```text
legged_rl_controller/legged_rl_controller_plugins.xml
```

插件名是：

```text
legged_rl_controller/LeggedRLController
```

## 机器人描述和 launch

Go2 描述包：

```text
legged_robot_description/go2_description
```

G1 描述包：

```text
legged_robot_description/g1_description
```

每个描述包通常包含：

- `urdf/`：URDF/Xacro 和 `ros2_control` 描述。
- `config/ros2_control/`：controller manager 和 controller 参数。
- `config/main_loop/`：手柄、controller 切换和速度缩放配置。
- `launch/`：bringup、broadcaster、static、RViz 等启动入口。
- `rviz2/`：可视化配置。

Go2 还包含：

- `config/rl_policy/policy.onnx`
- `config/rl_policy/IO_descriptors.yaml`
- `config/mapping/` 中的 TF 配置。

## RL 数据流

RL 控制器的数据流是：

```text
joint states + IMU + cmd_vel
        |
ObservationManager
        |
ONNX Runtime policy inference
        |
action term
        |
joint position commands
```

这里最关键的约束是 joint order。`rl_controller.joint_names`、`IO_descriptors.yaml` 中的 joint 顺序、policy action 输出顺序必须保持一致。

## Mapping 辅助工具

`legged_mapping` 不替代 Livox driver 或 FAST-LIO。它主要解决外部 LIO 和机器人 TF 树之间的 frame 衔接问题。

当前建图流程使用 `ground_odom_tf_node`：节点根据启动时脚点位置拟合地面 odom，并发布 `odom -> initial_base` 等 TF。

典型 FAST-LIO 链路：

```text
odom -> initial_base -> camera_init -> body -> base
```

其中 `camera_init -> body` 由 FAST-LIO 发布，其余静态或初始化 TF 由本仓库辅助节点提供。

## 设计边界

这个仓库应该负责：

- 机器人状态和命令接口。
- controller 插件和 RL 策略运行。
- 机器人描述、控制配置和 bringup。
- 与机器人控制强相关的 TF/感知辅助。

这个仓库不应该直接承担：

- 完整 SLAM 算法实现。
- 高层导航规划。
- 训练 RL policy 的完整训练框架。
- Unitree 官方 SDK 或仿真器本身的维护。
