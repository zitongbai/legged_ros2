# legged_ros2 中文文档

`legged_ros2` 是一个面向腿式机器人的 ROS 2 控制、仿真和实机部署仓库。当前代码以 ROS 2 Humble 和 `ros2_control` 为核心，提供 Unitree Go2、Unitree G1 的机器人描述、硬件接口、控制器插件、RL 策略运行框架，以及建图和动捕相关辅助工具。

这份中文文档是新的内容入口。它不按旧文档逐页翻译，而是按使用和开发任务重新组织，便于后续迁移成文档网站。

## 推荐阅读路径

如果你是第一次使用：

1. 阅读[项目概览](getting-started/overview.md)，理解仓库解决的问题和当前支持范围。
2. 优先按[Docker 安装与运行](getting-started/docker-installation.md)准备环境；需要本机开发时再看[安装与环境配置](getting-started/installation.md)。
3. 使用[Go2 快速启动](getting-started/quick-start-go2.md)跑通一个最小流程。
4. 根据场景阅读 [Go2 仿真运行](usage/simulation-go2.md) 或 [Go2 实机运行](usage/real-robot-go2.md)。

如果你要改代码或接入新机器人：

1. 阅读[系统架构](architecture/system-overview.md)。
2. 阅读[RL 控制定制](rl/customization.md)。
3. 阅读[新增机器人](robots/add-new-robot.md)。
4. 遇到问题时查看[常见问题](reference/common-issues.md)。

如果你要做建图：

1. 阅读[Go2 + MID360 + FAST-LIO 建图](mapping/go2-mid360-fast-lio.md)。

## 当前文档结构

- `getting-started/`：项目概览、安装、快速启动。
- `usage/`：仿真、实机和常用运行方式。
- `architecture/`：包职责、运行时数据流和设计边界。
- `rl/`：ONNX 策略、IO 描述、观察量和动作接口。
- `mapping/`：Go2 + MID360 + FAST-LIO 和地面 odom TF。
- `robots/`：机器人描述和新增机器人流程。
- `reference/`：命令、排查清单和常见问题。

## 支持范围

当前仓库中已经存在的主要包包括：

- `legged_ros2_control`：`ros2_control` 运行时、硬件接口和 Unitree Go2/G1 后端。
- `legged_ros2_controller`：经典控制器插件，例如静态姿态控制器。
- `legged_rl_controller`：基于 ONNX Runtime 的 RL 控制器插件。
- `legged_robot_description/go2_description`：Go2 的 URDF/Xacro、launch、控制配置和 RL policy 资源。
- `legged_robot_description/g1_description`：G1 的 URDF/Xacro、launch 和控制配置。
- `legged_mapping`：MID360/FAST-LIO 使用中的静态 TF 和地面 odom TF 辅助工具。
- `legged_mocap`：基于 `libmotioncapture` 的动捕 Pose 发布节点。

## 安全提示

实机运行前必须确认机器人周围环境安全，并熟悉急停流程。`enable_lowlevel_write:=true` 会允许控制器向底层发送低层电机命令；调试机器人描述、TF、状态广播或建图时，应优先使用 broadcaster-only 或关闭低层写入的模式。
