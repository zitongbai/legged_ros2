# 常用命令

本页记录开发和运行中常用命令。默认从 workspace 根目录 `~/legged_ws` 执行。

## 环境

```bash
source /opt/ros/humble/setup.bash
source path/to/unitree_ros2/setup_local.sh
source install/setup.bash
```

实机：

```bash
source path/to/unitree_ros2/setup.sh
source install/setup.bash
```

仿真：

```bash
export ROS_DOMAIN_ID=1
```

## 构建

全部构建：

```bash
colcon build --symlink-install
```

单包构建：

```bash
colcon build --symlink-install --packages-select legged_ros2_control
```

RL 相关构建：

```bash
colcon build --symlink-install --packages-select legged_rl_controller go2_description
```

## 测试

```bash
colcon test --packages-select legged_mapping
colcon test-result --verbose
```

## Go2 启动

broadcaster-only：

```bash
ros2 launch go2_description bringup_broadcasters.launch.py
```

静态控制：

```bash
ros2 launch go2_description bringup_static.launch.py use_rviz:=true use_rqt_cm:=true
```

RL 控制：

```bash
ros2 launch go2_description bringup_rl.launch.py use_rviz:=true use_rqt_cm:=true
```

## G1 启动

```bash
ros2 launch g1_description bringup_broadcasters.launch.py
ros2 launch g1_description bringup_static.launch.py use_rviz:=true use_rqt_cm:=true
ros2 launch g1_description bringup_rl.launch.py use_rviz:=true use_rqt_cm:=true
```

G1 的 RL policy 路径可以通过 launch 参数覆盖：

```bash
ros2 launch g1_description bringup_rl.launch.py \
  onnx_model_path:=/absolute/path/to/policy.onnx \
  io_descriptors_path:=/absolute/path/to/IO_descriptors.yaml
```

## controller 检查

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 control list_hardware_components
```

## topic 检查

```bash
ros2 topic list
ros2 topic echo /joint_states
ros2 topic echo /cmd_vel
```

## TF 检查

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo odom base
```

## Mapping

Go2 地面 odom TF：

```bash
ros2 launch go2_description ground_odom_tf.launch.py
```

通用 ground odom TF：

```bash
ros2 launch legged_mapping ground_odom_tf.launch.py \
  params_file:=/absolute/path/to/ground_odom_tf.yaml
```

## Motion Capture

```bash
ros2 launch legged_mocap mocap_pose.launch.py \
  mocap_type:=nokov \
  hostname:=192.168.50.95 \
  frame_id:=world
```

输出 topic 形式：

```text
/mocap/<rigid_body>/pose
```
