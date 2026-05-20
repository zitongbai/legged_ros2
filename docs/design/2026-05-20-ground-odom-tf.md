# 地面 odom 静态 TF 设计

日期：2026-05-20

## 背景

原有 `lidar_static_tf_node` 发布两段静态 TF：

```text
odom -> camera_init
body -> base
```

其中 `camera_init -> body` 由 FAST-LIO 发布。当前配置下，`odom -> camera_init` 和 `body -> base` 的标定值基本互为逆变换；当 FAST-LIO 初始 `camera_init -> body` 接近单位变换时，整条链路初始近似为：

```text
odom -> base ~= identity
```

因此旧 `odom` 的实际语义接近“启动时刻的 base 坐标系”，而不是地面坐标系。

需要保留旧标定程序输出的数值，因为这部分本质上是“初始机体坐标系到 LIO 初始坐标系”的标定结果。为了避免把标定外参和地面 odom 定义混在一起，新设计引入显式中间坐标系 `initial_base`。

## 目标 TF 链路

新增 `ground_odom_tf_node` 后，目标链路为：

```text
odom -> initial_base -> camera_init -> body -> base
```

各 frame 语义如下：

- `odom`：新的地面 odom，原点在初始脚点几何中心附近，xy 平面贴合初始脚点平面。
- `initial_base`：启动初始化时刻的 `base` 坐标系，也是旧 `odom` 的实际语义。
- `camera_init`：FAST-LIO 的初始 tracking origin。
- `body`：FAST-LIO 发布的动态 tracking body。
- `base`：机器人当前 base frame。

原有两段标定中的第一段从：

```yaml
odom_to_tracking_origin
```

重命名为：

```yaml
initial_base_to_tracking_origin
```

数值保持不变，只更新语义和 parent frame。

## 参数设计

默认参数文件：

```text
legged_mapping/config/ground_odom_tf.yaml
legged_robot_description/go2_description/config/mapping/ground_odom_tf.yaml
```

关键参数：

```yaml
ground_odom.parent_frame: "odom"
ground_odom.child_frame: "initial_base"
ground_odom.base_frame: "base"
ground_odom.foot_frames: ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]
ground_odom.foot_radius: 0.02
ground_odom.tf_timeout_sec: 10.0
```

标定外参保留为：

```yaml
initial_base_to_tracking_origin.parent_frame: "initial_base"
initial_base_to_tracking_origin.child_frame: "camera_init"
initial_base_to_tracking_origin.translation_xyz: [0.183746, -0.008833, 0.079768]
initial_base_to_tracking_origin.rotation_angles: [-2.059313, 12.900457, -0.424709]
```

`tracking_body_to_base` 仍然表示：

```text
body -> base
```

## `odom -> initial_base` 计算方法

节点启动后等待所有配置的脚部 TF 可用：

```text
base -> FL_foot
base -> FR_foot
base -> RL_foot
base -> RR_foot
```

设每个脚 link 原点在 `base` 坐标系下的位置为：

```text
p_i
```

### 1. 脚点数量检查

`ground_odom.foot_frames` 至少需要 3 个脚点。少于 3 个点无法定义平面，节点直接启动失败。

### 2. 平面拟合

当前实现按机器人正常站立的使用场景，拟合近水平平面：

```text
z = ax + by + c
```

并由此得到平面法向：

```text
n = normalize([-a, -b, 1])
```

如果法向和初始 `base` 的 z 轴反向，则翻转：

```text
if dot(n, base_z) < 0:
    n = -n
```

这样 `odom` 的 z 轴总是尽量朝向初始 `base` 的上方。

### 3. odom 原点

先计算所有脚 link 原点的几何中心：

```text
c = mean(p_i)
```

由于 Go2 的 foot link 原点对应球形脚的球心，默认启用 `foot_radius`，将 odom 原点沿平面法向下移：

```text
o = c - foot_radius * n
```

如果后续其他机器人不需要这个补偿，可以把 `ground_odom.foot_radius` 设为 `0.0`。

### 4. odom 朝向

为了保持初始 `base` 的 yaw，使用初始 `base` 的 x 轴投影到脚平面上作为 `odom` 的 x 方向：

```text
x = base_x - dot(base_x, n) * n
x = normalize(x)
```

如果 `base_x` 投影退化，则使用 `base_y` 投影作为备用。

之后构造右手正交坐标系：

```text
z = n
y = normalize(cross(z, x))
x = normalize(cross(y, z))
```

其中 `x, y, z` 都是在 `initial_base/base` 坐标系下表达的 `odom` 坐标轴。

### 5. 发布方向

由 `x, y, z` 和原点 `o` 可构造：

```text
T_initial_base_odom
```

但 TF 中实际发布的是：

```text
odom -> initial_base
```

因此程序发布其逆变换：

```text
T_odom_initial_base = inverse(T_initial_base_odom)
```

例如水平站立时，脚球心高度约为 `z = -0.30`，`foot_radius = 0.02`，则地面 odom 原点在 base 下约为 `z = -0.32`，最终发布：

```text
odom -> initial_base translation ~= [0, 0, 0.32]
```

## 启动顺序

`ground_odom_tf_node` 需要读取脚部 link 的 TF，因此必须在 `robot_state_publisher` 和 joint state/broadcaster 已经工作后启动。

推荐顺序：

```text
Livox driver
FAST-LIO
bringup_broadcasters
ground_odom_tf_node
```

脚本 `scripts/run_mapping_terminals.sh` 已按此顺序调整。

## 失败策略

以下情况节点会直接报错退出，不发布错误 TF：

- `ground_odom.foot_frames` 少于 3 个。
- 任意脚部 TF 在 `ground_odom.tf_timeout_sec` 内不可用。
- 脚点平面拟合退化。
- `base` x/y 轴无法投影到脚平面。
- 角度单位、旋转顺序或静态变换参数非法。

## 测试与验证

已添加基础 launch test：

```text
legged_mapping/test/test_ground_odom_tf.launch.py
```

测试通过 fake static TF 模拟 4 个脚点，验证：

- 节点发布 `odom -> initial_base`。
- 节点保留 `initial_base -> camera_init` 标定变换。
- 水平脚平面和 `foot_radius = 0.02` 时，`odom -> initial_base` 高度符合预期。

本地验证命令：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select legged_mapping --symlink-install
colcon test --packages-select legged_mapping --event-handlers console_direct+
colcon test-result --verbose
```

真实机器人或完整 mapping 流程验证建议：

```bash
source /root/legged_ws/setup.sh
ros2 launch go2_description bringup_broadcasters.launch.py
```

另一个终端：

```bash
source /root/legged_ws/setup.sh
ros2 launch go2_description ground_odom_tf.launch.py
```

也可以使用整套 mapping 脚本：

```bash
export NET_IF=<your-network-interface>
bash /root/legged_ws/src/legged_ros2/scripts/run_mapping_terminals.sh
```
