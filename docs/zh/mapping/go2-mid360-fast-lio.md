# Go2 + MID360 + FAST-LIO 建图

`legged_mapping` 不直接实现 SLAM。它提供 `ground_odom_tf_node`，帮助 Go2 + MID360 + FAST-LIO 形成一致的地面 odom 和机器人 TF 树。

典型建图链路：

```text
MID360 -> livox_ros_driver2 -> FAST-LIO -> TF
Go2 state -> robot_state_publisher -> robot TF
legged_mapping -> LIO frame 和 robot base 的静态/初始化 TF
```

最终 TF 链路为：

```text
odom -> initial_base -> camera_init -> body -> base
```

其中：

- `odom`：地面 odom，xy 平面贴合启动时脚点平面。
- `initial_base`：启动初始化时刻的 `base` 坐标系。
- `camera_init`：FAST-LIO 的初始 tracking origin。
- `body`：FAST-LIO 发布的动态 tracking body。
- `base`：机器人当前 base frame。

## 硬件连接

典型连接：

```text
PC <-- Ethernet --> Go2 <-- Cable --> MID360
```

MID360 通过网络通信。PC 可以直接接收 MID360 数据，也可以使用 Livox Viewer 2 修改雷达网络参数。

## MID360 网络参考配置

如果使用 Unitree 提供并已经配置好的 MID360，通常可以跳过重新配置。否则可参考：

```text
Lidar IP:       192.168.123.20
Gateway:        192.168.123.1
Lidar Info IP:  192.168.123.70
```

Livox driver 的 host IP 需要和 PC 连接 Go2/MID360 的网卡一致。

## 本机依赖

需要在 PC 上安装：

- Livox-SDK2
- livox_ros_driver2
- FAST-LIO ROS 2 版本

建议将 `livox_ros_driver2` 和 FAST-LIO 放在独立 workspace 中，避免依赖冲突。例如：

```text
~/livox_ws
~/fast_lio_ws
~/legged_ws
```

构建 FAST-LIO 前，先 source `livox_ws/install/setup.bash`。

## Docker mapping 环境

仓库提供 mapping 专用 Docker：

```bash
cd ~/legged_ws/src/legged_ros2
docker/build_mapping.sh
docker/run_mapping.sh
```

进入已有容器：

```bash
docker/enter_mapping.sh
```

mapping 容器内包含：

- `/root/livox_ws`
- `/root/fast_lio_ws`
- `/root/legged_ws`

运行前设置连接机器人/MID360 的网卡：

```bash
export NET_IF=<your-local-network-interface>
```

例如：

```bash
export NET_IF=enp3s0
```

## 启动流程

通常需要四个终端。

终端 1：启动 Livox driver。

```bash
export NET_IF=<your-local-network-interface>
source /root/legged_ws/setup.sh
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}
source /root/livox_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

终端 2：启动 FAST-LIO。

```bash
export NET_IF=<your-local-network-interface>
source /root/legged_ws/setup.sh
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}
source /root/livox_ws/install/setup.bash
source /root/fast_lio_ws/install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

终端 3：启动 Go2 状态和 TF。

```bash
export NET_IF=<your-local-network-interface>
source /root/legged_ws/setup.sh
ros2 launch go2_description bringup_broadcasters.launch.py
```

终端 4：启动地面 odom TF。

```bash
export NET_IF=<your-local-network-interface>
source /root/legged_ws/setup.sh
ros2 launch go2_description ground_odom_tf.launch.py
```

## 辅助脚本

容器中可以使用：

```bash
export NET_IF=<your-local-network-interface>
bash /root/legged_ws/src/legged_ros2/scripts/run_mapping_terminals.sh
```

该脚本使用 `xterm` 打开多个终端，并按推荐顺序启动 mapping 相关节点。

## TF 目标

`ground_odom_tf_node` 发布并维护建图所需的静态/初始化 TF。目标链路：

```text
odom -> initial_base -> camera_init -> body -> base
```

其中：

- `camera_init -> body` 由 FAST-LIO 动态发布。
- `odom -> initial_base` 由脚点平面初始化得到。
- `initial_base -> camera_init` 是标定外参。
- `body -> base` 是 LIO body 到机器人 base 的外参。

## 参数文件

通用默认参数：

```text
legged_mapping/config/ground_odom_tf.yaml
```

Go2 参数：

```text
go2_description/config/mapping/ground_odom_tf.yaml
```

Go2 launch：

```bash
ros2 launch go2_description ground_odom_tf.launch.py
```

也可以直接启动 `legged_mapping` 的通用 launch：

```bash
ros2 launch legged_mapping ground_odom_tf.launch.py \
  params_file:=/absolute/path/to/ground_odom_tf.yaml
```

## 关键参数

```yaml
ground_odom.parent_frame: "odom"
ground_odom.child_frame: "initial_base"
ground_odom.base_frame: "base"
ground_odom.foot_frames: ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]
ground_odom.foot_radius: 0.02
ground_odom.tf_timeout_sec: 10.0
```

标定外参：

```yaml
initial_base_to_tracking_origin.parent_frame: "initial_base"
initial_base_to_tracking_origin.child_frame: "camera_init"
```

LIO body 到机器人 base：

```yaml
tracking_body_to_base.parent_frame: "body"
tracking_body_to_base.child_frame: "base"
```

## 地面 odom 初始化

`ground_odom_tf_node` 启动后等待脚部 link 的 TF：

```text
base -> FL_foot
base -> FR_foot
base -> RL_foot
base -> RR_foot
```

至少需要 3 个脚点，否则无法定义平面。

当前实现按近水平站立场景拟合：

```text
z = ax + by + c
```

得到平面法向后，如果法向和初始 `base` 的 z 轴反向，则翻转法向，保证 `odom` 的 z 轴尽量朝向机器人上方。

`odom` 原点取脚点几何中心，并沿平面法向向下移动 `foot_radius`，用于补偿 Go2 球形脚 link 原点到地面的半径。

`odom` 朝向尽量保留初始 `base` 的 yaw：将初始 `base` 的 x 轴投影到脚点平面作为 `odom` x 方向，再构造右手正交坐标系。

## 启动顺序

`ground_odom_tf_node` 需要读取脚部 TF，因此必须在 robot state publisher 和 joint state/broadcaster 已经工作后启动。

推荐顺序：

```text
Livox driver
FAST-LIO
bringup_broadcasters
ground_odom_tf_node
```

## 失败策略

以下情况节点会报错退出，不发布错误 TF：

- `ground_odom.foot_frames` 少于 3 个。
- 任意脚部 TF 在超时时间内不可用。
- 脚点平面拟合退化。
- base x/y 轴无法投影到脚平面。
- 角度单位、旋转顺序或静态变换参数非法。
