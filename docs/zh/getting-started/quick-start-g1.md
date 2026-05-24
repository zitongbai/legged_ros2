# G1 快速启动

> 目前G1的流程尚未调通，请勿使用

当前仓库已经包含 `g1_description` 和 `legged_ros2_control` 中的 G1 硬件接口实现。G1 的使用方式和 Go2 类似，但具体 policy、网络和安全流程需要按你的 G1 环境确认。

## 主要文件

G1 description：

```text
legged_robot_description/g1_description
```

G1 硬件接口：

```text
legged_ros2_control/src/robots/unitree_g1
```

主要 launch：

```text
g1_description/launch/bringup_broadcasters.launch.py
g1_description/launch/bringup_static.launch.py
g1_description/launch/bringup_rl.launch.py
g1_description/launch/view_robot.launch.py
```

## 查看模型

```bash
source /opt/ros/humble/setup.bash
source ~/legged_ws/install/setup.bash
ros2 launch g1_description view_robot.launch.py
```

## 启动 broadcaster-only

先启动只读状态广播：

```bash
source path/to/unitree_ros2/setup.sh
source ~/legged_ws/install/setup.bash
ros2 launch g1_description bringup_broadcasters.launch.py
```

检查：

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

## 启动静态控制

```bash
ros2 launch g1_description bringup_static.launch.py use_rviz:=true use_rqt_cm:=true
```

第一次实机测试时，应先确认 broadcaster-only 正常，再尝试 static controller。

## 启动 RL 控制

```bash
ros2 launch g1_description bringup_rl.launch.py use_rviz:=true use_rqt_cm:=true
```

G1 的 `bringup_rl.launch.py` 支持通过 launch 参数指定 policy：

```bash
ros2 launch g1_description bringup_rl.launch.py \
  onnx_model_path:=/absolute/path/to/policy.onnx \
  io_descriptors_path:=/absolute/path/to/IO_descriptors.yaml
```

这两个文件必须来自同一次导出，并和 G1 的 joint order 对齐。

## 注意事项

- G1 自由度和 Go2 不同，不能直接使用 Go2 policy。
- `rl_controller.joint_names` 必须和 G1 policy descriptor 中的关节顺序一致。
- 实机运行前确认 Unitree G1 底层通信、网络、急停和安全支撑。
- 如果只是检查模型或状态，优先使用 `bringup_broadcasters.launch.py`。
