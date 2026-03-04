# Legged RL Controller Waypoint Command 数据流设计

日期：2026-03-04  
范围：`legged_rl_controller` 部署侧将 `generated_commands` 从 `cmd_vel(3)` 切换为 `waypoint_window(3 * N)`。

## 1. 目标与约束

- 目标：在部署侧复用训练时 `generated_commands` 语义，直接消费 `go2_path` 发布的 waypoint window。
- 输入话题：`/waypoints`，消息类型 `geometry_msgs/msg/PoseArray`。
- 关键约束：
  - `generated_commands` 名称保持不变（对齐 IO descriptors 与 ONNX 输入约定）。
  - 旧的 `cmd_vel` 实现代码保留但禁用（通过注释，不删除）。
  - 无消息时 `generated_commands` 全部置零。
  - `PoseArray` 长度与 `num_waypoints` 不一致时直接报错。
  - `num_waypoints` 从 `IO_descriptors.yaml` 推断，不靠手工配置。

## 2. 训练语义对齐

- 训练侧 `generated_commands` 本质是 command manager 输出的 command tensor。
- 对 waypoint_window 命令，command 语义为展平向量：
  - `[w0_x, w0_y, w0_z, w1_x, w1_y, w1_z, ...]`
  - 维度 `generated_command_dim = 3 * num_waypoints`
- 部署侧仅负责接收与透传该“采样后命令”，不负责 path 构建。

## 3. IO 描述符驱动的维度推断

在控制器 `on_configure()` 解析 IO 描述符时执行：

1. 读取 `observations` 中 `name == generated_commands` 的配置项。  
2. 读取其 `shape[0]`，记为 `generated_command_dim`。  
3. 校验：
   - `generated_command_dim > 0`
   - `generated_command_dim % 3 == 0`
4. 推断：
   - `num_waypoints = generated_command_dim / 3`
5. 任何校验失败均直接返回配置错误（拒绝启动）。

## 4. 单一状态源（ArticulationData）

将 waypoint 相关运行时状态统一保存在 `ArticulationData`，作为全局共享数据源：

- `num_waypoints`
- `generated_command_dim`
- `path_command`（`std::vector<float>`，长度固定为 `generated_command_dim`）

设计原则：

- Controller 负责初始化这些字段。
- Articulation 负责每周期更新 `path_command`。
- Observation 只读取，不重复推断维度。

## 5. 数据流

1. `on_configure()`：
   - 读取 IO descriptors，推断维度并写入 `robot_->data`。
   - 初始化 `path_command` 全零。
   - 创建 `PoseArray` 订阅与实时缓冲。

2. `on_activate()`：
   - 重置 waypoint 缓冲。
   - 将 `path_command` 重新置零。

3. `LeggedArticulation::update()`（每控制周期）：
   - 读取最新 `PoseArray`。
   - 无消息：`path_command` 保持/写为全零。
   - 有消息：校验 `poses.size() == num_waypoints`，不一致直接抛错。
   - 一致时按 `(x,y,z)` 顺序展平写入 `path_command`。

4. `generated_commands` observation：
   - 直接返回 `path_command`。
   - 旧 `cmd_vel` 版本保留为注释块，明确“已停用”。

## 6. 与 go2_path 的接口约定

- 订阅消息默认来自 `/waypoints`。
- waypoint 坐标默认已在 `base_yaw_only_frame` 下，控制器不再做坐标变换。
- `num_waypoints` 的最终一致性由：
  - 训练导出的 `IO_descriptors.yaml`（维度约束）
  - `go2_path` 发布长度（运行时一致性）
  共同保证。

## 7. 失败策略

- IO descriptors 缺失或维度非法：`on_configure()` 失败。
- 运行中收到长度不匹配的 `PoseArray`：当前周期报错并返回失败，防止错误输入进入策略。
- 无消息场景：输入全零，系统可安全启动和等待上游命令。

