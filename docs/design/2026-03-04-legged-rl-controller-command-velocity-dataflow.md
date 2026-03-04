# Legged RL Controller 中 Command Velocity 数据流设计

日期：2026-03-04  
范围：`legged_rl_controller` 包内 `cmd_vel`（`geometry_msgs/msg/Twist`）相关链路

## 1. 设计目标

将外部速度指令（`linear.x`, `linear.y`, `angular.z`）稳定送入 RL 策略观测，保证：

- 非实时 ROS 回调与实时控制循环解耦；
- 策略始终可获取最新命令（无命令时回退为零）；
- 命令范围在观测入口处可配置裁剪。

## 2. 数据流总览

1. 控制器参数声明并读取 `cmd_vel_topic` 与 `cmd_vel_range_*`。  
2. 创建 `Twist` 订阅，将最新消息写入 `RealtimeBuffer`。  
3. 每个控制周期 `env_->step()` 先调用 `robot->update()`。  
4. `LeggedArticulation::update()` 从 `RealtimeBuffer` 读取最新 `Twist`。  
5. 写入 `robot->data.velocity_command.{lin_vel_x, lin_vel_y, ang_vel_z}`。  
6. 观测项 `generated_commands` 读取三维命令并按 `cmd_vel_range_*` 执行 clamp。  
7. ObservationManager 按配置顺序拼接观测，送入 ONNX 输入（通常为 `obs`/`policy` 组）。

## 3. 模块职责

### 3.1 `LeggedRLController`

- 负责参数生命周期与 ROS 接口：
  - `cmd_vel_topic`（默认 `cmd_vel`）
  - `cmd_vel_range_lin_vel_x`
  - `cmd_vel_range_lin_vel_y`
  - `cmd_vel_range_ang_vel_z`
- 在 `on_configure()` 中：
  - 初始化 `cmd_vel_buffer_`；
  - 创建 `Twist` 订阅，回调内执行 `writeFromNonRT(msg)`；
  - 将参数范围写入 `robot_->data.velocity_command.range.*`。
- 在 `on_activate()` 中重置 `cmd_vel_buffer_`，使激活初期命令为安全零值。

### 3.2 `LeggedArticulation`

- 在 `update()` 中读取 `*cmd_vel_buffer_->readFromRT()`：
  - 若为空指针：`lin_vel_x = 0`, `lin_vel_y = 0`, `ang_vel_z = 0`；
  - 否则直接映射：
    - `lin_vel_x <- msg->linear.x`
    - `lin_vel_y <- msg->linear.y`
    - `ang_vel_z <- msg->angular.z`
- 该步骤不做滤波、不做斜坡限制，仅同步“最新值”。

### 3.3 `generated_commands` 观测项

- 从 `robot->data.velocity_command` 取 3 维命令；
- 以 `velocity_command.range.*` 为上下界执行 `std::clamp`；
- 输出固定维度 `3` 的观测向量。

## 4. 与策略输入的对齐

- `IO_descriptors.yaml` 中 `observations.policy`（或单组映射到 `obs`）定义了观测项顺序；
- `generated_commands` 作为其中一项，参与拼接后进入 ONNX 模型输入；
- 若该项配置 `history_length: 5`，则该 3 维命令会被历史缓冲展开为 `3 * 5` 参与输入。

## 5. 时序与运行特性

- 控制循环顺序：`robot->update() -> observation_manager->compute() -> alg->act(obs)`；
- 因此同一周期内，策略看到的是“本周期刚读取”的最新命令；
- 在未收到任何 `cmd_vel` 或激活后缓冲刚重置时，策略输入命令为全零。

## 6. 当前边界与风险点

- 无通信超时机制：只要缓冲中有旧消息，就会持续使用旧值；
- 无低通滤波/加速度限制：命令突变会直接进入策略观测；
- 范围限制仅在观测侧 clamp，不会反馈修改外部命令源。

## 7. 结论

当前设计采用“`Twist` 最新值 + 实时缓冲解耦 + 观测端裁剪”的轻量链路，结构清晰、实时友好，适合与 IsaacLab 导出的策略观测接口直接对接。对于更强安全性和可控性，可在后续引入命令超时、滤波与斜坡限制机制。
