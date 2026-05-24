# 新增机器人

本页说明如何把新的腿式机器人接入 `legged_ros2`。推荐先阅读当前 Go2 和 G1 的实现，再按相同边界扩展。

新增机器人通常涉及四类内容：

1. 机器人描述包。
2. `legged_ros2_control` 硬件接口。
3. controller 和 main loop 配置。
4. 可选的 RL policy、mapping、mocap 配置。

## 创建 description package

建议在 `legged_robot_description/` 下创建新的 description package，例如：

```text
legged_robot_description/<your_robot>_description
```

典型目录：

```text
<your_robot>_description/
  CMakeLists.txt
  package.xml
  urdf/
  meshes/
  config/
    ros2_control/
    main_loop/
    rl_policy/
  launch/
  rviz2/
```

`CMakeLists.txt` 需要安装资源目录：

```cmake
install(
  DIRECTORY config launch meshes urdf rviz2
  DESTINATION share/${PROJECT_NAME}
)
```

## URDF/Xacro

至少需要：

- 机器人本体模型。
- joint/link 名称。
- mesh 路径。
- IMU link。
- `ros2_control` 标签。
- 硬件插件名。
- command/state interface 定义。

检查点：

- URDF 中的 joint 名称必须和 controller YAML 一致。
- `ros2_control` 中的 sensor 名称必须和 controller 中的 `imu_names` 一致。
- mesh 使用 `package://<description_pkg>/meshes/...` 路径。
- xacro 可以独立展开。

验证：

```bash
colcon build --symlink-install --packages-select <your_robot>_description
source install/setup.bash
xacro path/to/your_robot.urdf.xacro > /tmp/your_robot.urdf
```

可视化：

```bash
ros2 launch <your_robot>_description view_robot.launch.py
```

## ros2_control 配置

在 `config/ros2_control/` 中准备不同运行模式的 YAML。

建议至少提供：

```text
broadcasters_only.yaml
static.yaml
rl.yaml
```

`broadcasters_only.yaml` 用于只读状态和 TF，默认应关闭底层写入，适合调试机器人描述和通信。

`static.yaml` 用于加载站立/坐下等静态姿态控制器。

`rl.yaml` 用于加载 RL controller，并配置：

- `onnx_model_path`
- `io_descriptors_path`
- `joint_names`
- `imu_names`
- `cmd_vel_topic`
- `cmd_vel_range_*`

## main loop 配置

在 `config/main_loop/` 中配置手柄和 controller 切换。

典型内容：

```yaml
<robot>_wireless_controller:
  ros__parameters:
    cmd_vel:
      scale:
        lin_vel_x: 1.0
        lin_vel_y: 0.8
        ang_vel_z: 1.0
    controller_bindings:
      stand_static_controller: 1
      sit_static_controller: 2
      rl_controller: 3
      off: 9
```

具体节点名取决于你的 main loop 和 wireless controller 实现。

## 硬件接口

如果新机器人已有可复用的通信方式，可以参考 Go2/G1 后端实现。

代码位置建议：

```text
legged_ros2_control/src/robots/<your_robot>/
```

典型文件：

```text
<your_robot>_lowlevel_node.cpp
<your_robot>_system_interface.cpp
<your_robot>_wireless_controller.cpp
<your_robot>_main_loop.cpp
CMakeLists.txt
```

硬件接口需要完成：

- 订阅底层状态。
- 发布底层命令。
- 将 URDF joint 映射到底层电机索引。
- 在 `read()` 中填充 position、velocity、effort 和 IMU。
- 在 `write()` 中写入目标关节命令。
- 处理 CRC 或 SDK 特有字段。
- 支持 `enable_lowlevel_write`，方便调试时只读不写。

## 注册硬件插件

更新：

```text
legged_ros2_control/legged_ros2_control_plugins.xml
```

示例形式：

```xml
<library path="<your_robot>_system_interface">
  <class name="legged_ros2_control/<YourRobot>SystemInterface"
         type="legged::<YourRobot>SystemInterface"
         base_class_type="legged::LeggedSystemInterface">
    <description>ROS2 system interface for your robot</description>
  </class>
</library>
```

同时更新 `legged_ros2_control/CMakeLists.txt`，加入新机器人子目录。

## launch 文件

建议按 Go2/G1 提供：

```text
bringup_broadcasters.launch.py
bringup_static.launch.py
bringup_rl.launch.py
view_robot.launch.py
```

launch 需要组合：

- `robot_description`
- controller YAML
- main loop YAML
- main loop executable
- robot state publisher
- controller spawner
- RViz/rqt 可选参数

## RL policy 接入

如果机器人要运行 RL：

```text
config/rl_policy/policy.onnx
config/rl_policy/IO_descriptors.yaml
```

要求：

- 两个文件来自同一次 Isaac Lab 导出。
- `rl_controller.joint_names` 顺序和 descriptor 中机器人关节顺序一致。
- action 维度等于受控关节数。
- IMU 名称存在并可用。

不要手工改 descriptor 来绕过顺序或维度问题。

## 最小验证流程

1. 构建 description 和 control：

```bash
colcon build --symlink-install --packages-select \
  legged_ros2_control legged_ros2_controller legged_rl_controller <your_robot>_description
```

2. 只启动 broadcaster：

```bash
ros2 launch <your_robot>_description bringup_broadcasters.launch.py
```

3. 检查接口：

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

4. 启动 static controller。

5. 最后再启动 RL controller。

## 常见失败原因

- joint 名称或顺序不一致。
- `ros2_control` 插件名和 XML 注册名不一致。
- IMU sensor 名称不一致。
- 底层 SDK 消息和 joint 映射错误。
- controller YAML 中配置了不存在的 controller type。
- ONNX 和 descriptor 不匹配。
