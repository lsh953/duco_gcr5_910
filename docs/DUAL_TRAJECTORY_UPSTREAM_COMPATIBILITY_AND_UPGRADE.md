# 双轨迹 Action 上游接口兼容性说明与升级说明

- 适用分支：`integration/round3-safety-fixes`（个人仓库 `lsh953/duco_gcr5_910`）
- 集成基线：已审阅分支 `codex/dual-trajectory-safety-fixes` = `31b2eaa`
- 上游基准：`lsh953/main` = `7e7c34b`
- 审计环境：ROS 2 Jazzy，`ros-jazzy-moveit` **2.12.3**，`control_msgs` Jazzy
- 审计方式：安装版 MoveIt 的源码级核对（GitHub `moveit/moveit2` tag `2.12.3`）＋本仓库全量 grep

本文回答三类问题：**取消了取消超时报告为 CANCELED 以后，上游会看到什么**、
**上游到底读了 Result 的哪些字段**、以及**本次交付包含的全部行为/参数变更与迁移动作**。

---

## 0. 结论摘要

| # | 结论 | 影响等级 |
|---|---|---|
| 1 | 取消超时由 `CANCELED` 改为 `ABORTED`（error_code 仍为 `GOAL_TOLERANCE_VIOLATED=-5`） | **上游可见的行为变更**，必须通知 |
| 2 | MoveIt 2.12.3 **同时读取**终止状态、`Result.error_code`、`Result.error_string` | 已核对源码，见 §2.2 |
| 3 | 该变更使 MoveIt 侧的最终错误码由 `PREEMPTED(-7)` 变为 `CONTROL_FAILED(-4)` | **上位机需确认分支逻辑** |
| 4 | MoveIt 自身**不会**在 ABORTED 后自动重发轨迹 | 不需要改 MoveIt 配置 |
| 5 | 本轮之前 `Result.error_string` 一直为空，MoveIt 的告警行因此丢掉了原因 | 本轮已补齐（§3.2） |
| 6 | 仓库内唯一的自研 Action Client `send_joint_waypoints.py` 只读 `error_code`，失败也返回退出码 0 | 第 4 轮已修复（§2.3） |
| 7 | 上位机/客户 HMI 的 Action Client **不在本仓库内**，无法核对 | **必须由上位机侧确认**，见 §2.5 |
| 8 | 软件联锁触发后，任何"自动重试"都会被拒绝，但**这不是硬件安全功能** | 见 §4 |

---

## 1. 行为变更一：取消超时不再报告 CANCELED

### 1.1 变更内容

| 场景 | 变更前 | 变更后 |
|---|---|---|
| 取消请求被 Gazebo 接收，但在 `cancel_timeout` 内**始终没有**终止状态回报 | `CANCELED` + `GOAL_TOLERANCE_VIOLATED` | **`ABORTED`** + `GOAL_TOLERANCE_VIOLATED` |
| 取消请求被 Gazebo**明确拒绝** | `ABORTED` | `ABORTED`（不变） |
| Gazebo 正常完成、取消输掉竞争 | 按真实结果上报（成功/失败） | 不变 |
| Gazebo 正常响应取消 | `CANCELED` | 不变 |

代码位置：`src/duco_ros_driver/include/duco_ros_driver/DualTrajectoryAction.hpp`
`watchdogCallback()` 的 `TaskState::Canceling && now >= cancel_deadline_` 分支。

### 1.2 判定依据

1. ROS 2 的取消协议里，"服务器接受了 CancelGoal 请求"**只**表示它承诺尝试取消；
   真正的结论只能由该目标的终止状态给出。
2. 取消超时意味着**没有任何终止状态**，因此"是否已经停下"在软件层面不可知。
3. `action_msgs/GoalStatus` 中 `STATUS_CANCELED = 5` 的语义是"该目标被取消了"。
   在只知道"取消请求被接收"的情况下上报 5，等于**把未知当作已停止**，
   这与实机联锁的整个前提（未知即锁定）自相矛盾。
4. 合法性：`rcl_action` 允许 `CANCELING → ABORTED`（也允许 `CANCELING → CANCELED`），
   因此该变更不违反 action 状态机，客户端不会收到非法状态序列。

### 1.3 变更前后，客户端看到的字节级差异

| 观察点 | 变更前 | 变更后 |
|---|---|---|
| Action 终止状态 (`action_msgs/GoalStatus`) | `STATUS_CANCELED = 5` | `STATUS_ABORTED = 6` |
| `Result.error_code` | `-5` | `-5`（不变） |
| `Result.error_string` | 变更前为空 | 非空，例如 "cancellation timed out without any cancellation result from Gazebo; the robot may still be executing" |
| MoveIt `ExecutionStatus` | `PREEMPTED` | `ABORTED` |
| MoveIt `MoveItErrorCodes` | `PREEMPTED = -7` | `CONTROL_FAILED = -4` |
| MoveIt 日志 | `Controller '...' reports status PREEMPTED`（WARN） | `Controller '...' failed with error GOAL_TOLERANCE_VIOLATED: <原因>`（WARN） |

**含义**：变更前上位机看到的是"你的取消生效了"，变更后看到的是"这次执行失败了，
而且**没有任何证据表明机械臂已停止**"。这正是本轮修复要表达的语义。

---

## 2. 上游调用方审计

### 2.1 审计范围

| 类别 | 对象 | 是否在仓库内 |
|---|---|---|
| MoveIt 规划执行 | `move_group` → `moveit_simple_controller_manager` | 否（系统安装 2.12.3） |
| 本仓库轨迹发送脚本 | `src/duco_gcr5_910_moveit_config/scripts/send_joint_waypoints.py` | 是 |
| 孪生验证脚本 | `scripts/dual_twin_verification/twin_common.py`、`phase_*.py` | 是（**测试工具，非生产**） |
| 真实命令消费方 | `src/duco_ros_driver/src/DucoDriver.cpp`（订阅 `joint_path_command`） | 是（厂商驱动） |
| 上位机 / 客户 HMI / PLC | — | **否（无法核对）** |

### 2.2 MoveIt 2.12.3（主要上游，已源码核对）

调用链：

```
MoveGroupInterface / RViz / moveit_commander
   └─ move_group (ExecuteTrajectory / MoveGroup action)
        └─ TrajectoryExecutionManager::executePart()
             └─ FollowJointTrajectoryControllerHandle      (moveit_simple_controller_manager)
                  └─★ 本节点 DualTrajectoryAction 的 front action
```

**★ 处的结果处理**（`moveit_plugins/moveit_simple_controller_manager/src/follow_joint_trajectory_controller_handle.cpp`）：

```cpp
void FollowJointTrajectoryControllerHandle::controllerDoneCallback(const WrappedResult& wrapped_result)
{
  if (!wrapped_result.result) {
    RCLCPP_WARN_STREAM(logger_, "Controller '" << name_ << "' done, no result returned");
  } else if (wrapped_result.result->error_code == Result::SUCCESSFUL) {
    RCLCPP_INFO_STREAM(logger_, "Controller '" << name_ << "' successfully finished");
  } else {
    RCLCPP_WARN_STREAM(logger_, "Controller '" << name_ << "' failed with error "
        << errorCodeToMessage(wrapped_result.result->error_code) << ": "
        << wrapped_result.result->error_string);          // ← 读取 error_string
  }
  finishControllerExecution(wrapped_result.code);          // ← 读取终止状态
}
```

**终止状态映射**（`action_based_controller_handle.hpp`）：

| Action 终止状态 | MoveIt `ExecutionStatus` |
|---|---|
| `SUCCEEDED` | `SUCCEEDED` |
| **`ABORTED`** | **`ABORTED`** |
| `CANCELED` | `PREEMPTED` |
| `UNKNOWN` | `UNKNOWN` |

**最终上报给 move_group 客户端的错误码**（`moveit_ros/move_group/src/default_capabilities/execute_trajectory_action_capability.cpp`）：

| `ExecutionStatus` | `moveit_msgs/MoveItErrorCodes` |
|---|---|
| `SUCCEEDED` | `SUCCESS = 1` |
| `PREEMPTED` | `PREEMPTED = -7` |
| `TIMED_OUT` | `TIMED_OUT = -6` |
| 其它（含 `ABORTED`） | **`CONTROL_FAILED = -4`** |

**是否自动重试**：`TrajectoryExecutionManager::executePart()` 在
`handle->getLastExecutionStatus() != SUCCEEDED` 时 `result = false`，
`executeThread()` 随即**停止执行后续轨迹段**并把状态返回给调用方；
`trajectory_execution_manager.cpp` 中唯一的 `retry` 循环（第 1095 行附近）是
"重新加载控制器列表"，与重发轨迹无关。**结论：MoveIt 不会自动重新下发轨迹。**

**已读字段汇总**：

| 上游 | 终止状态 | `error_code` | `error_string` | 自动重试 |
|---|---|---|---|---|
| MoveIt 2.12.3 | ✅ | ✅ | ✅（写进 WARN 日志） | ❌ |
| `send_joint_waypoints.py` | ✅（第 4 轮补上） | ✅ | ✅（第 4 轮补上） | ❌ |
| `twin_common.py`（测试） | ✅ | ✅ | ❌ | ❌（Phase A 的重试只针对"目标被拒绝"） |
| 上位机 | **未知** | **未知** | **未知** | **未知** |

### 2.3 仓库内 Action Client：`send_joint_waypoints.py`（**第 4 轮已修复**）

`src/duco_gcr5_910_moveit_config/scripts/send_joint_waypoints.py` 现在的行为：

- `send_goal_async` → 等结果 → **同时**判定终止状态、`error_code` 与 `error_string`；
- 只有 `GoalStatus.STATUS_SUCCEEDED` **且** `error_code == SUCCESSFUL` 才返回退出码 0；
- 其余全部返回非零退出码：无 Action Server、目标被拒绝、ABORTED、CANCELED、
  UNKNOWN/未知状态、`error_code` 非 `SUCCESSFUL`、Result 对象缺失、YAML/路径点非法、
  通信异常；
- 失败日志包含失败原因、`action_status`、`error_code`、`error_string`，不是裸数字；
- **没有**自动重试，也**不会**自动发送下一条轨迹；一次调用只发一个目标。

CLI 参数（`waypoints_file` / `--action` / `--wait-sec`）、YAML 路径点格式、轨迹合法性检查、
默认 Action 名（`/arm_1_controller/follow_joint_trajectory`）全部保持不变。

> **注意（语义边界）**：退出码 0 只表示"Action Server 确认轨迹成功执行完毕"，
> **不表示**任何硬件级停止确认；非零退出码也不表示机械臂已经停下——
> 取消超时等情况下真实执行结果未知，必须由人工按联锁恢复流程处理。

对应回归测试：`src/duco_gcr5_910_moveit_config/test/test_send_joint_waypoints.py`
（63 个用例，含一个进程内模拟 Action Server 的端到端组；见第 4 轮交接文件）。

### 2.4 孪生验证脚本（测试工具，不影响生产）

`scripts/dual_twin_verification/twin_common.py` 读取终止状态与 `error_code`，
Phase B 的 S2 用例断言 `status=5`（CANCELED）——**这是正确的**，因为孪生里由真实的
`joint_trajectory_controller` 应答取消，走的是"正常取消"路径，不经过取消超时。
取消超时/取消被拒两条路径只由 §3.3 的 mock 集成测试覆盖。

`phase_a_check.py` 的重试是对"目标被拒绝"的启动竞态重试，最长 4 次后失败退出。
它**不能**绕过联锁：联锁生效时 `handle_goal()` 持续拒绝，重试只会得到新的拒绝。

### 2.5 上位机 / 客户 HMI / PLC（**必须由上位机侧确认**）

- 本仓库内**没有**上位机 Action Client 代码，无法核对。
- `plc_s7_bridge` / `plc_modbus_bridge` 不订阅 `FollowJointTrajectory`，
  不消费该 Action，**因此不受本次变更影响**。
- 需要上位机负责人确认的四件事：
  1. 是否依据**终止状态**（`STATUS_CANCELED=5` vs `STATUS_ABORTED=6`）分支；
  2. 是否同时读取 `Result.error_code` 与 `Result.error_string`；
  3. 是否存在"失败/中止后自动重发轨迹"或"自动下发下一条轨迹"的逻辑；
  4. 是否把"目标被拒绝"当作可无限重试的瞬时故障。
     （联锁生效时 `handle_goal()` 会持续拒绝新目标，无限重试只会刷日志。）

---

## 3. Result 字段契约

### 3.1 `GOAL_TOLERANCE_VIOLATED` 用于取消超时的语义歧义（已在本文档显式定义）

`GOAL_TOLERANCE_VIOLATED = -5` 的字面含义是"未在容差内到达目标位姿"。
本节点把它用于**四种**互不相同的结局：

| 结局 | 终止状态 | error_code | `error_string` 关键词 |
|---|---|---|---|
| 真实反馈未在容差内到达（`wait_for_real` 下确认失败） | `ABORTED` | `-5` | 反馈/容差相关 |
| Gazebo 轨迹执行超时 | `ABORTED` | `-5` | `Gazebo trajectory execution timed out` |
| 真实执行超时 | `ABORTED` | `-5` | `real robot execution timed out` |
| **取消超时（本文档主题）** | `ABORTED` | `-5` | `cancellation timed out ... may still be executing` |
| 取消被拒 | `ABORTED` | `-5` | `cancellation timed out and Gazebo had rejected the cancel request` |

**因此 `error_code` 不能单独用来区分这几种结局，尤其不能用来推断"机械臂已停止"。**

- 需要区分时，**必须**读取 `Result.error_string`（本轮已填充），或订阅诊断话题
  `dual_trajectory/real_motion_state` 的 `reason=` / `state=` / `latched=` 字段。
- 保留 `-5` 而不新造码的原因：`FollowJointTrajectory` 的码表是上游固定枚举，
  其余取值（`INVALID_GOAL`、`INVALID_JOINTS`、`OLD_HEADER_TIMESTAMP`、
  `PATH_TOLERANCE_VIOLATED`）在这里语义更差；而终止状态 `ABORTED` 已经承担了
  "这次执行失败了"的主语义。该取舍是刻意的，并在此显式记录。

### 3.2 本轮补齐：`Result.error_string` 现在携带原因

`control_msgs/action/FollowJointTrajectory.action` 对 `error_string` 的原文定义是：

> "Human readable description of the error code. Contains complementary
> information that is especially useful when execution fails"

而 §2.2 已确证 MoveIt 会把它打进 WARN 日志。本轮之前该字段**始终为空**，
导致 MoveIt 的告警行变成 `failed with error GOAL_TOLERANCE_VIOLATED: `（冒号后无内容），
丢失了"取消超时 / 真实执行超时 / 反馈未确认"的区分。

变更（`finishGoal()`）：终止状态不为 `SUCCEEDED` 时填充 `error_string`
（内容等于该目标的中止原因；若原因缺失则回落到锁存原因），
`SUCCESSFUL` 时保持为空，避免只读该字段的调用方把成功误判为失败。

**契约**：

| 终止状态 | `error_code` | `error_string` |
|---|---|---|
| `SUCCEEDED` | `0` | **空** |
| 其它 | 非 `0` | **非空**，人类可读原因 |

### 3.3 兼容性测试证据

`src/duco_ros_driver/test/test_dual_trajectory_action.cpp`：

| 用例 | 断言 |
|---|---|
| `CancelTimeoutIsReportedAsAbortedNotCanceled` | 终止状态 `ABORTED`；`error_code == -5`；`error_string` 非空、含 `timed out`、含 `may still be executing` |
| `CancelTimeoutWithRealCommandLatchesAndBlocksNewGoals` | 超时后终止状态 `ABORTED`；下一个目标被拒绝；落盘记录存在 |
| `RejectedCancelIsNotReportedAsACleanCancellation` | 取消被拒 ⇒ 不是干净取消 |
| `CancelLosingTheRaceReportsTheRealOutcome` | 取消与正常完成竞争 ⇒ 按真实结果上报 |
| `CancelWhileExecutingIsForwardedToGazebo` | 正常取消仍为 `CANCELED`（未回归） |
| `RepeatedCancelRequestsTerminateTheGoalOnce` | 重复取消只终止一次 |

---

## 4. 联锁与"自动重试"的相互作用

1. 只要**已经向真实通道发布过轨迹**，且该目标不是"已确认的真实完成"，
   结束时就会把真实运动联锁置为锁定态（`onGoalFinishedUnconfirmed`），
   取消超时/执行超时进一步置为**不确定**（`latchUncertainLocked`）。
2. 锁定态下 `handle_goal()` **拒绝**新的真实目标；拒绝会体现在
   `Result.error_string` 与节点日志中。
3. 因此，上游若实现"失败自动重发"，重发**不会**绕过联锁，只会得到一连串拒绝。
   但**拒绝是软件层面的**——本联锁**不是**硬件安全功能，
   不得替代安全回路、安全 PLC、使能开关或急停。
4. 解除锁定**只能**由操作员调用 `clear_fault` 服务（`std_srvs/Trigger`），
   且需要机器人状态健康、反馈新鲜等客观条件同时成立。

> **升级要求**：上位机必须把"目标被拒绝"与"轨迹执行失败"当作**需要人工介入**的
> 状态处理，不得实现无上限的自动重试，也不得用"重发成功"来证明机械臂已停下。

---

## 5. 升级说明：本次交付包含的全部行为/参数变更

### 5.1 实机与仿真默认隔离

| 项 | 内容 |
|---|---|
| 相关参数 | `use_real_robot`（launch，默认 `false`）、`publish_real`（节点，默认 `false`）、`wait_for_real`（默认 `false`）、`fault_recovery_enabled`（默认 `false`） |
| 变更 | 默认**不发布**任何真实命令、默认不等待真实完成、默认关闭人工恢复服务 |
| 影响 | 只启动仿真时，行为与纯仿真一致，不会触碰真机 |
| 迁移 | 要接触真机必须**显式**同时给出 `use_real_robot:=true`（并在需要时 `wait_for_real:=true`） |

### 5.2 `wait_for_real=false` 时，实体运动完成仍需单独确认

| 项 | 内容 |
|---|---|
| 变更 | 此时 Action 可以按仿真结果报 `SUCCEEDED`，但该结果**不代表**机械臂到达了目标；真实联锁仍保持锁定，`real=` 记为 `unconfirmed` |
| 影响 | 客户端可能收到 `SUCCEEDED`，而真实通道仍被锁，后续真实目标被拒绝 |
| 迁移 | 需要真实完成保证时，显式设置 `wait_for_real:=true`，并保证真实反馈/状态话题可用且新鲜 |

### 5.3 实机异常联锁后必须人工恢复

| 项 | 内容 |
|---|---|
| 相关参数 | `latch_on_real_fault`（默认 `true`）、`fault_recovery_enabled`（默认 `false`）、`clear_fault_service`（默认 `clear_fault`）、`real_motion_state_topic` |
| 变更 | 真实异常/未确认运动触发后，必须由操作员调用 `clear_fault` 服务解锁；服务默认**关闭**，需显式启用 |
| 影响 | 未启用恢复服务时，锁定态只能通过重启并保持记录、再由操作员处理 |
| 迁移 | 现场须明确谁是操作员、在何处调用该服务；**服务本身不是安全认证**，它只是"操作员已检查现场"的声明 |

### 5.4 反馈超时参数改名（**破坏性**：旧名会导致启动报错）

| 旧名 | 新名 | 语义 |
|---|---|---|
| `real_feedback_timeout` | `real_feedback_max_age` | 真实反馈样本的最大允许年龄 |
| `real_status_timeout` | `real_status_max_age` | 真实机器人状态样本的最大允许年龄 |

旧名未声明会以"未声明参数"启动失败（**不会**被静默忽略）。
launch 文件与孪生脚本已同步；外部启动文件/上位机参数列表需同步改名。

### 5.5 真实机械臂地址必须显式配置

| 项 | 内容 |
|---|---|
| 相关参数 | `server_host_1`（launch，默认 `""`） |
| 变更 | 历史上写死的 `192.168.6.4` 已被移除；`use_real_robot:=true` 而未给 `server_host_1` 时 launch **直接抛错** |
| 迁移 | 每次实机启动必须显式传入现场实际地址，禁止依赖历史默认值 |

### 5.6 本轮（round 3）新增/变化的参数汇总

| 参数 | 默认值 | 说明 |
|---|---|---|
| `fault_latch_file` | **`""`**（原 `/tmp/duco_dual_trajectory_fault.state`） | 空 ⇒ 用内置默认：`$XDG_STATE_HOME` 或 `~/.local/state` 下的 `duco/dual_trajectory_interlock.state`。**不再使用共享临时目录**（会被清理/重启清空，等于抹掉未确认运动的记忆） |
| `real_persistence_required` | **`true`** | 为真时，联锁状态文件必须可写，否则**拒绝发布真实轨迹** |
| 诊断话题 `dual_trajectory/real_motion_state` | — | 新增 `persistence=ok\|unavailable` 字段；`state=`、`latched=`、`sim=`、`real=`、`reason=` 含义见 §3.1 |

### 5.7 兼容性矩阵（本轮）

| 项 | 结论 |
|---|---|
| Action 名称 / Action 类型 | 未变 |
| Gazebo 集成 | 未变（round 3 孪生端到端 PASS） |
| MoveIt 集成 | **客户端可见错误码在取消超时路径由 `-7` 变为 `-4`**；其余不变 |
| 反馈校验、目标代号（generation） | 未变 |
| 既有参数语义 | 除 §5.4 改名外未变 |
| 新增参数默认值 | `fault_latch_file=""`、`real_persistence_required=true` 均为**更保守**的默认值 |

---

## 6. 升级检查清单

- [ ] 上位机：确认按终止状态分支的代码能处理 `STATUS_ABORTED = 6`。
- [ ] 上位机：确认 `MoveItErrorCodes.CONTROL_FAILED = -4` 与 `PREEMPTED = -7` 都被当成失败处理。
- [ ] 上位机：确认不存在"失败后自动重发轨迹 / 自动下发下一条轨迹"。
- [ ] 上位机：确认不会把"目标被拒绝"当作可无限重试的瞬时故障。
- [ ] 上位机：确认没有被 `GOAL_TOLERANCE_VIOLATED = -5` 单独解释为"停在容差外"。
- [x] 运维：`send_joint_waypoints.py` 的退出码已修复（第 4 轮，§2.3）；无需再跟进。
- [ ] 运维：确认外层脚本/调度器按非零退出码停止后续下发，且**不要**实现"失败自动重发"。
- [ ] 部署：启动参数里的 `real_feedback_timeout` / `real_status_timeout` 是否已改名（§5.4）。
- [ ] 部署：`server_host_1` 是否已显式配置（§5.5）。
- [ ] 部署：确认 `fault_latch_file` 指向的位置在启动后**可写**，并检查诊断话题里 `persistence=ok`（§5.6）。
- [ ] 现场：确认 `clear_fault` 服务的责任人、调用方式与记录方式（§5.3）。
- [ ] 培训：确认所有相关方都理解"软件联锁 ≠ 硬件安全功能"。

---

## 7. 已知限制

1. **无厂商停止确认接口**：`controller_connected_flag_` / `robotmoving()` 未发布，
   停止沿用"空 `JointTrajectory`"约定。因此取消超时后**无法**由软件证明机械臂已停止。
2. 上位机 Action Client 不在本仓库内，§2.5 的四项确认**未经核对**。
3. `GOAL_TOLERANCE_VIOLATED` 的语义重载（§3.1）是刻意的取舍，不是遗漏。
4. `send_joint_waypoints.py` 的退出码缺口已在第 4 轮修复（§2.3），
   仓库内已无已知的"失败返回 0"Action Client。
5. 孪生 Phase B 的 S2 只覆盖"正常取消"路径；取消超时/取消被拒只由 mock 集成测试覆盖。
