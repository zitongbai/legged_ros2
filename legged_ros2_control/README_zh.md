# legged_ros2_control

## 简介

`legged_ros2_control` 是本仓库中用于对接 `ros2_control` 的硬件接口 package，主要面向 Unitree 机器人。

它的作用是将机器人底层的电机、IMU 和遥控器数据封装到 `ros2_control` 框架中，使上层控制器可以通过标准接口访问机器人状态并下发控制命令。

这个 package 不仅提供 `hardware_interface::SystemInterface` 插件，还额外实现了一个自定义的 `ros2_control` 运行入口，用于手动创建并维护：

- `ResourceManager`
- `ControllerManager`
- 控制循环线程
- 遥控器输入桥接节点

因此，它可以看作是一个针对腿足机器人和 Unitree 平台定制的 `ros2_control` 运行时。

## 主要职责

这个 package 主要负责以下几件事：

- 解析 URDF 中 `ros2_control` 标签描述的硬件信息
- 创建并导出关节和 IMU 的状态接口
- 创建并导出关节命令接口
- 将 `ros2_control` 的接口映射到 Unitree 底层 ROS 2 topic
- 运行 `read -> update -> write` 控制循环
- 将无线遥控器输入转换为 `/cmd_vel` 和控制器切换请求

## 目录结构

核心文件大致可以分为以下几类：

- 抽象硬件接口
  - `include/legged_ros2_control/legged_system_interface.hpp`
  - `src/legged_system_interface.cpp`
- 自定义 ros2_control 运行封装
  - `include/legged_ros2_control/legged_ros2_control.hpp`
  - `src/legged_ros2_control.cpp`
- Go2 相关实现
  - `include/legged_ros2_control/robots/unitree_go2/`
  - `src/robots/unitree_go2/`
- G1 相关实现
  - `include/legged_ros2_control/robots/unitree_g1/`
  - `src/robots/unitree_g1/`
- 插件导出描述
  - `legged_ros2_control_plugins.xml`

## 架构说明

### 1. 抽象基类 `LeggedSystemInterface`

`LeggedSystemInterface` 继承自 `hardware_interface::SystemInterface`，是整个 package 的核心抽象层。

它的主要职责是：

- 从 `HardwareInfo` 中读取 joint 和 sensor 信息
- 为每个关节维护状态缓存和命令缓存
- 导出标准 joint state interface
  - `position`
  - `velocity`
  - `effort`
- 导出额外的状态接口
  - `position_des`
  - `velocity_des`
- 导出 joint command interface
  - `position`
  - `velocity`
  - `effort`
  - `kp`
  - `kd`
- 导出 IMU 接口
  - `orientation`
  - `angular_velocity`
  - `linear_acceleration`

这说明上层控制器并不是仅发送力矩命令，而是支持 Unitree 常见的混合控制格式：

- 目标位置 `q`
- 目标速度 `dq`
- 位置增益 `kp`
- 速度增益 `kd`
- 前馈力矩 `tau`

### 2. 具体机器人实现

在抽象基类之上，这个 package 目前实现了两个具体机器人硬件接口：

- `Go2SystemInterface`
- `G1SystemInterface`

这两个类都继承自 `LeggedSystemInterface`，主要完成以下工作：

- 将 URDF 中的关节名称映射到底层 SDK 使用的电机索引
- 在 `read()` 中从 Unitree 底层状态消息读取关节和 IMU 数据
- 在 `write()` 中将控制器输出写入 Unitree 低层命令消息

从结构上看，Go2 和 G1 的实现高度一致，只是底层消息类型、关节索引映射和 CRC 处理略有不同。

### 3. 低层 ROS 2 通讯节点

以 Go2 为例，`Go2LowLevelNode` 负责与 Unitree ROS 2 topic 对接：

- 订阅 `/lowstate`
- 发布 `/lowcmd`
- 在发送前计算 CRC

`Go2SystemInterface::read()` 会先调用 `rclcpp::spin_some()` 处理这个 node 的回调，然后把最新的 low state 拷贝到 `joint_data_` 和 `imu_data_`。

`Go2SystemInterface::write()` 则将控制器输出写入 low cmd，再发布到底层。

G1 的实现也是同样的模式。

### 4. 自定义控制运行时 `LeggedRos2Control`

这个 package 没有直接依赖官方默认的 `ros2_control_node`，而是实现了一个 `LeggedRos2Control` 类，手动完成以下流程：

1. 获取 `robot_description`
2. 解析 URDF 中的 `ros2_control` 配置
3. 创建 `ResourceManager`
4. 通过 `pluginlib` 动态加载对应的硬件接口插件
5. 创建 `ControllerManager`
6. 启动独立线程运行控制循环
7. 启动 executor 处理 ROS 回调

控制循环本质上就是：

```text
read() -> update() -> write()
```

这样设计的原因是为了更灵活地控制初始化流程以及实时相关配置，例如：

- update rate
- CPU affinity
- memory lock
- 实时线程优先级

对于腿足机器人，这类控制通常对时序较为敏感，因此这种做法比完全依赖默认启动节点更容易做定制。

## 插件机制

硬件接口插件通过 `pluginlib` 导出，定义在 `legged_ros2_control_plugins.xml` 中。

当前导出了两个插件：

- `legged_ros2_control/Go2SystemInterface`
- `legged_ros2_control/G1SystemInterface`

这意味着在 URDF 的 `ros2_control` 配置中，只要指定相应的 `hardware_class_type`，`ResourceManager` 就可以动态加载对应实现。

## 数据流

整个 package 的数据流可以概括为：

```text
Unitree lowstate topic
    -> Go2/G1LowLevelNode
    -> Go2/G1SystemInterface::read()
    -> ros2_control state interfaces
    -> controllers
    -> ros2_control command interfaces
    -> Go2/G1SystemInterface::write()
    -> Unitree lowcmd topic
```

如果加入无线遥控器节点，则还有另一条辅助链路：

```text
WirelessController topic
    -> Go2WirelessController / G1WirelessController
    -> /cmd_vel
    -> controller_manager/switch_controller
```

## Go2 主循环入口

Go2 的可执行文件 `go2_main_loop` 会：

1. 初始化 ROS 2
2. 创建普通 node
3. 创建 `LeggedRos2Control`
4. 调用 `init()` 建立整个 ros2_control 运行时
5. 创建无线遥控器节点
6. 在主线程中 spin 遥控器节点

也就是说，这个 package 既负责硬件适配，也负责把整个控制运行环境真正拉起来。

## 无线遥控器节点的作用

Go2 和 G1 都带有无线遥控器桥接逻辑，主要做两件事：

- 将摇杆轴值映射到 `/cmd_vel`
- 通过 `controller_manager/switch_controller` 服务切换控制器

例如，某些按键组合会触发：

- 激活某个控制器
- 关闭其他控制器
- 关闭全部控制器

这让机器人在不同控制模式之间切换更加方便，尤其适合实验和调试场景。

## 关键参数和行为

### `enable_lowlevel_write`

具体硬件接口类中支持 `enable_lowlevel_write` 参数。

- 为 `true` 时，会真的向机器人发送低层控制命令
- 为 `false` 时，只读取状态，不下发控制

默认值为 `false`。

这个设计对真实机器人比较重要，可以降低误下发命令的风险。

### IMU 数量限制

当前实现默认只支持一个 IMU。

如果在硬件描述中配置了多个 IMU，初始化会报错。

### 自定义接口

除了标准的 `position`、`velocity`、`effort` 外，这个 package 还额外定义并导出了：

- `kp`
- `kd`
- `position_des`
- `velocity_des`

因此，上层控制器如果要完整使用这套控制链路，需要知道这些接口名称并按约定访问。

## 构建产物

从 `CMakeLists.txt` 可以看出，这个 package 主要构建以下内容：

- 抽象基类库 `legged_system_interface`
- Go2 硬件插件库 `go2_system_interface`
- G1 硬件插件库 `g1_system_interface`
- Go2 主程序 `go2_main_loop`
- G1 主程序 `g1_main_loop`

这说明它既是一个库 package，也是一个可执行入口 package。

## 依赖关系

主要依赖包括：

- `rclcpp`
- `hardware_interface`
- `controller_manager`
- `pluginlib`
- `rclcpp_lifecycle`
- `urdf`
- `rosgraph_msgs`
- `unitree_go`
- `unitree_hg`

其中 `unitree_go` 和 `unitree_hg` 表明这个 package 明显是面向 Unitree 生态的专用实现，而不是通用硬件接口框架。

## 总结

`legged_ros2_control` 可以理解为：

> 一个针对腿足机器人，特别是 Unitree Go2 / G1 的 `ros2_control` 硬件接口与运行时封装。

它解决的问题包括：

- 如何把底层机器人状态接入 `ros2_control`
- 如何把控制器输出映射到底层电机命令
- 如何运行一套自定义的控制循环
- 如何通过手柄实现速度命令和控制模式切换

如果你要继续阅读这个仓库，建议下一步重点看：

- 机器人描述包中的 `ros2_control` 配置
- `legged_ros2_controller/` 中控制器如何声明和消费这些接口
- `legged_rl_controller/` 中 RL 控制器如何接入这套硬件层
