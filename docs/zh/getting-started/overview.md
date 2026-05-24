# 项目概览

`legged_ros2` 的目标是把腿式机器人的硬件通信、`ros2_control` 控制器、机器人描述、RL policy 推理和外部感知辅助工具组织成一套可复用的 ROS 2 栈。

这个仓库不是单一 demo。它更像是一套可扩展框架：底层通过硬件接口连接真实机器人或仿真器，中间通过 `controller_manager` 管理控制器，上层通过 launch 和 YAML 组合不同运行模式。

## 核心能力

- 通过 `ros2_control` 暴露关节和 IMU 接口。
- 支持 Unitree Go2 和 G1 的硬件接口插件。
- 提供静态姿态控制、RL 控制等 controller 插件。
- 支持加载 Isaac Lab 导出的 ONNX policy 和 `IO_descriptors.yaml`。
- 提供 Go2/G1 的机器人描述、控制器配置和 bringup launch。
- 提供 Go2 + MID360 + FAST-LIO 使用时需要的地面 odom TF 辅助节点。
- 提供 motion capture pose 发布节点。

## 运行环境

当前文档以以下环境为基准：

- Ubuntu 22.04
- ROS 2 Humble
- `ros2_control`
- Unitree ROS 2 / Unitree SDK 相关消息和通信环境
- ONNX Runtime 1.22.0

## 主要包职责

`legged_ros2_control`

负责启动控制主循环、加载硬件接口、连接 Unitree 底层消息、执行 `read -> update -> write`。Go2 使用 `go2_main_loop`，G1 使用 `g1_main_loop`。

`legged_ros2_controller`

提供非 RL 的 controller 插件，包括 `StaticController`、`RandomTrajController`、`SeparateJointController` 等。常用的站立、坐下姿态由 `StaticController` 配置出来。

`legged_rl_controller`

提供 `LeggedRLController` 插件。它读取关节状态、IMU 和速度指令，构造 observation，调用 ONNX Runtime 执行策略推理，再把 action 转换成关节命令。

`legged_robot_description`

存放机器人模型和运行配置。Go2 和 G1 各自有 description package，包含 URDF/Xacro、RViz 配置、launch 文件、`ros2_control` YAML、main loop YAML，以及 policy 资源。

`legged_mapping`

不直接实现 SLAM。它为外部 LIO 系统提供 `ground_odom_tf_node`，典型使用场景是 Go2 搭载 MID360，通过 Livox driver 和 FAST-LIO 运行建图。

`legged_mocap`

连接 motion capture 后端，将每个刚体的姿态发布为 `/mocap/<rigid_body>/pose`。

## 推荐先跑通的路径

对新用户，推荐先用 Go2 跑通下面路径：

1. 安装 ROS 2 Humble、Unitree ROS 2 和本仓库依赖。
2. 下载 ONNX Runtime 到 `third_party/`。
3. 构建 workspace。
4. 在 Unitree Mujoco 中运行 Go2 仿真。
5. 启动 `go2_description bringup_rl.launch.py`。
6. 通过手柄切换站立、坐下和 RL 控制器。

实机运行需要额外确认网络、DDS、低层控制服务和安全空间，不能直接照搬仿真流程。
