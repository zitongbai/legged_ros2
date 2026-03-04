# Legged RL Controller Waypoint Command 实施计划

日期：2026-03-04  
对应设计文档：`docs/design/2026-03-04-legged-rl-controller-waypoint-generated-commands-design.md`

## 1. 实施目标

- 将部署侧 `generated_commands` 从 `cmd_vel(3)` 切换为 waypoint window (`3 * N`)。
- `N`（`num_waypoints`）从 `IO_descriptors.yaml` 自动推断并保存到 `ArticulationData`。
- 保留旧 `cmd_vel` 代码但禁用（注释，不删除）。

## 2. 文件级改动计划

### 2.1 `legged_rl_controller/include/legged_rl_controller/isaaclab/assets/articulation/articulation.h`

- 新增 waypoint 命令状态字段（单一状态源）：
  - `num_waypoints`
  - `generated_command_dim`
  - `path_command`
- 保留现有 `velocity_command` 结构（不删除）。

### 2.2 `legged_rl_controller/include/legged_rl_controller/legged_rl_controller.hpp`

- 增加 waypoint 订阅相关类型与成员：
  - `PoseArray` 订阅句柄
  - `RealtimeBuffer<PoseArray::SharedPtr>`
- 保留 `cmd_vel` 成员定义（用于兼容，后续在 cpp 中注释禁用逻辑）。

### 2.3 `legged_rl_controller/src/legged_rl_controller.cpp`

- `on_init()`：
  - 增加 waypoint 话题参数声明（默认 `/waypoints`）。
  - 旧 `cmd_vel` 参数声明保留，但相关使用路径注释禁用。

- `on_configure()`：
  - 从 `IO_descriptors.yaml` 解析 `generated_commands.shape[0]`。
  - 校验维度合法：`>0` 且 `%3==0`，否则报错返回。
  - 推断 `num_waypoints = dim/3`，写入 `robot_->data`。
  - 初始化 `robot_->data.path_command` 为全零。
  - 创建 waypoint 订阅并写入 RT buffer。
  - 注释掉 `cmd_vel` 订阅创建与范围配置代码（不删除）。

- `on_activate()`：
  - reset waypoint RT buffer。
  - 显式把 `path_command` 清零。
  - `cmd_vel` reset 逻辑注释禁用（不删除）。

### 2.4 `legged_rl_controller/include/legged_rl_controller/legged_articulation.hpp`

- 构造函数改为接收 waypoint RT buffer。
- `update()` 中命令处理改为 waypoint：
  - 无消息：`path_command` 全零。
  - 有消息：
    - 校验 `poses.size() == num_waypoints`，不一致抛异常。
    - 按 `(x,y,z)` 展平写入 `path_command`。
- 原 `cmd_vel` 更新代码整体注释保留（不删除）。

### 2.5 `legged_rl_controller/include/legged_rl_controller/isaaclab/envs/mdp/observations/observations.h`

- `generated_commands` 改为返回 `path_command`。
- 保留旧速度版 `generated_commands` 实现为注释块，标注“停用，仅保留历史实现”。

### 2.6 构建配置

- `package.xml` / `CMakeLists.txt` 增加 `geometry_msgs` 依赖（若当前未声明为显式依赖）。

## 3. 关键校验点

- 配置期：
  - 找不到 `generated_commands` 或 `shape` 非法，控制器配置失败。
- 运行期：
  - `/waypoints` 未发布时，观测命令全零。
  - `/waypoints` 长度不匹配时，控制器报错停止该周期。
- 维度一致性：
  - `generated_command_dim == 3 * num_waypoints`
  - `path_command.size() == generated_command_dim`

## 4. 验收步骤

1. 使用新导出的 `policy.onnx + IO_descriptors.yaml` 启动 RL bringup。  
2. 查看日志确认：
   - 解析出的 `generated_command_dim` 与 `num_waypoints`。
   - waypoint 订阅已建立。  
3. 不发 `/waypoints` 时，系统保持运行且 command 为全零。  
4. 发送长度正确的 `PoseArray`，策略可正常运行。  
5. 发送长度错误的 `PoseArray`，控制器立即报错（符合预期）。  

## 5. 风险与缓解

- 风险：上游 `go2_path` 与 IO descriptors 维度不一致导致运行中断。  
  - 缓解：统一由同一实验配置生成并在启动前检查 topic 发布长度。  

- 风险：旧 `cmd_vel` 与新 waypoint 逻辑混用造成歧义。  
  - 缓解：保留但注释禁用 `cmd_vel` 路径，并在注释中写明停用原因。  

