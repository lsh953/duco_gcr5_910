# DUCO GCR5-910 双臂轨迹联动项目 —— 代码冻结交接文档

> 本文档由最后一次功能收尾轮生成，用于把当前开发版本完整、可追溯地交接给后续开发者。
> 所有结论都来自冻结后的 Git 提交与**实际执行过**的命令；未执行的项在 §8.6 显式列出，不做推测。

---

## 8.1 项目与版本信息

| 项 | 值 |
|---|---|
| 项目名称 | DUCO GCR5-910 双臂（单臂 6 轴）轨迹联动：数字孪生预演 + 实体机械臂受控执行 |
| 主要目标 | 用 Gazebo/MoveIt 数字孪生先验证轨迹，再在**显式开启**的前提下把轨迹交给实体机械臂，并保证实体通道的失败/超时/取消不会被静默忽略 |
| 工作区 | `/home/me/duco_ros2_ws` |
| 交接仓库（本次推送目标） | `https://github.com/lsh953/duco_gcr5_910`（git remote `lsh953`） |
| 机构/上游仓库 | `git@github.com:HENU-Makerspace/duco_gcr5_910_ros2_ws.git`（git remote `origin`） |
| 交接分支 | `codex/round3-code-freeze` |
| **代码冻结提交** | `454bb5a422eee0590a8bc3c722ff6f1b89ac8458` |
| 交接文档提交 | 本文件所在提交；`git log -1 --format=%H` 即为该分支最终提交 |
| 冻结候选标识 | `v1.0-code-freeze-candidate`（**仅开发候选标识**；未创建 Git Tag、未创建 Release，需项目负责人最终确认） |
| 上一轮评审基线 | `integration/round3-safety-fixes` = `1faf8f7a7b4f867b00a5229f1bffd75a2c1796f4` |
| 更早冻结分支 | `codex/dual-trajectory-safety-fixes` = `31b2eaa`（保持冻结，未改写） |
| 开发环境 | Ubuntu 24.04、ROS 2 Jazzy、MoveIt 2.12.3、Gazebo（ros_gz）、Python 3.12.3（`/usr/bin/python3`）、GCC 13.3.0 |
| 冻结日期 | 2026-10-17 |

**分支提交链（本分支自 `1faf8f7` 起）**

```
7e7c34b (lsh953/main)
  └─ … → 31b2eaa (codex/dual-trajectory-safety-fixes，已冻结)
          └─ … → 1faf8f7 (integration/round3-safety-fixes)
                  └─ 454bb5a (本分支：代码冻结提交)
                          └─ <交接文档提交> (codex/round3-code-freeze HEAD)
```

**本分支做过的改动（相对 `1faf8f7`）**

| 文件 | 性质 |
|---|---|
| `src/duco_gcr5_910_moveit_config/scripts/send_joint_waypoints.py` | 修复退出码语义（本轮唯一功能性修复） |
| `src/duco_gcr5_910_moveit_config/test/test_send_joint_waypoints.py` | **新增** 63 个回归用例 |
| `src/duco_gcr5_910_moveit_config/CMakeLists.txt` | 注册 pytest（显式选择解释器） |
| `src/duco_gcr5_910_moveit_config/package.xml` | 补 `action_msgs` / `control_msgs` 依赖与测试依赖 |
| `docs/DUAL_TRAJECTORY_UPSTREAM_COMPATIBILITY_AND_UPGRADE.md` | 修正“退出码缺口未修”的过期表述 |
| `docs/DUAL_TRAJECTORY_SAFETY_FIXES_ROUND3_INTEGRATION_REPORT.md` | 同上 |
| `docs/CODE_FREEZE_HANDOFF.md` | 本文档 |

未改动任何 C/C++ 源码、未改动任何既有 Action/Topic/Service 接口。

---

## 8.2 自研源码文件清单

行数与 SHA-256 均取自冻结提交 `454bb5a` 的文件内容（工作区哈希与提交内容一致，已用 `git status` 验证为干净）。
“依赖”一列指该文件直接包含/导入的接口，不含传递依赖。

### 8.2.1 轨迹接收与分发

| 路径 | 职责 | 关键类/函数 | 依赖 |
|---|---|---|---|
| `src/duco_ros_driver/include/duco_ros_driver/DualTrajectoryAction.hpp` | 前端动作服务器节点：接收上位机 `FollowJointTrajectory` 目标，先在孪生侧执行验证，再按开关决定是否下发实体通道；同时做任务状态机、联锁判定与状态发布（构造函数内联，全文件即节点实现） | `class DualTrajectoryAction : public rclcpp::Node`、`handle_accepted()`、`realFeedbackCallback()`、`realStatusCallback()`、`watchdogCallback()`、`validateConfiguration()`、`stateName()`、`goalIdText()` | `rclcpp`、`rclcpp_action`、`control_msgs/action/FollowJointTrajectory`、`trajectory_msgs/JointTrajectory`、`duco_msg/DucoRobotState`、`std_srvs/Trigger`、`builtin_interfaces` |
| `src/duco_ros_driver/src/DualTrajectoryAction.cpp` | 该节点的可执行入口（仅 `main()`，28 行） | `main()` | 上面的头文件 |

轨迹数据的解析、长度/时间戳/关节名一致性检查不在此节点内，而由 §8.2.3 的验证模块完成；节点只负责“调用验证 → 决定放行/拒绝”。

### 8.2.2 Action 任务管理与联锁状态机

| 路径 | 职责 | 关键类/函数 | 依赖 |
|---|---|---|---|
| `src/duco_ros_driver/include/duco_ros_driver/DualTrajectoryAction.hpp`（同上文件内） | 单一目标串行执行的任务状态机（IDLE/ACCEPTED/EXECUTING/…），目标 UUID 追踪，联锁状态锁定与发布，`clear_fault` 服务回调，故障标记恢复 | `enum TaskState`、`setStateLocked()`、`eraseSimRequestLocked()`、`restoreLatchedFault()`、`latchUncertainLocked()`、`latchRobotFaultLocked()`、`persistInterlockLocked()`、`removeFaultMarkerLocked()`、`publishInterlockStateLocked()`、`clearFaultCallback()`、`requestSimulationCancel()` | 同 8.2.1；持久化经 §8.2.6 模块 |

### 8.2.3 轨迹数据验证

| 路径 | 职责 | 关键类/函数 | 依赖 |
|---|---|---|---|
| `src/duco_ros_driver/include/duco_ros_driver/TrajectoryValidation.hpp` | 纯函数轨迹合法性检查：关节名非空/不重复、与期望关节名一致、路径点非空、每点位置数与关节数一致、数值有限、时间戳单调且严格递增、位置在软限位内；另含时间缩放 | `struct TrajectoryValidationPolicy`（`enforce_expected_joint_names`、`expected_joint_names`、`joint_position_min/max`）、`durationNanoseconds()`、`validateTrajectory()`（返回 `bool`，失败原因写入 `std::string & error`）、`scaleTrajectoryTime()` | 仅 STL + `builtin_interfaces`、`trajectory_msgs`（**不依赖 rclcpp**，因此可独立单元测试） |
| `src/duco_ros_driver/test/test_trajectory_validation.cpp` | 26 个用例 | — | gtest |

### 8.2.4 实体状态反馈校验

| 路径 | 职责 | 关键类/函数 | 依赖 |
|---|---|---|---|
| `src/duco_ros_driver/include/duco_ros_driver/RealFeedbackGate.hpp` | 判定“实体通道是否真的把轨迹执行到位”：要求状态/反馈序列号晚于下发时刻、时间戳未过期、`healthy` 为真、关节名匹配、位置误差在容差内、（可选）速度误差在容差内 | `struct FeedbackSnapshot`、`struct StatusSnapshot`、`struct GoalTarget`、`struct FeedbackPolicy`（`position_tolerance`、`velocity_tolerance`、`feedback_max_age`、`status_max_age`、`require_velocity`）、`enum class FeedbackVerdict`、`feedbackVerdictName()`、`evaluateRealFeedback()`（返回 `FeedbackVerdict`，逐项判定顺序即优先级） | 仅 STL（**不依赖 rclcpp**） |
| `src/duco_ros_driver/test/test_real_feedback_gate.cpp` | 39 个用例 | — | gtest |

### 8.2.5 运动异常联锁

| 路径 | 职责 | 关键类/函数 | 依赖 |
|---|---|---|---|
| `src/duco_ros_driver/include/duco_ros_driver/RealMotionInterlock.hpp` | 实体运动的状态与联锁：记录“已下发命令”的代数/目标号，只有在收到**确认完成**时才解除；未确认结束、不确定性、故障都会置位，并阻塞后续实体运动直到人工恢复 | `enum class RealMotionState`、`realMotionStateName()`、`realMotionSeverity()`、`class RealMotionInterlock`、`onCommandPublished()`、`onConfirmedCompletion()`、`onGoalFinishedUnconfirmed()`、`onUncertain()`、`onFault()`、`onOperatorRecovery()`、`escalate()`、`blocksRealMotion()`、`latched()`、`reset()` | 仅 STL（**不依赖 rclcpp**） |
| `src/duco_ros_driver/test/test_real_motion_interlock.cpp` | 14 个用例 | — | gtest |

> **边界（必须保留的表述）**：该联锁是**软联锁**，只控制本节点是否继续向实体通道下发命令，
> **不是**硬件安全功能，不等价于急停（E-Stop），也不能作为人身安全防护措施。

### 8.2.6 联锁状态持久化

| 路径 | 职责 | 关键类/函数 | 依赖 |
|---|---|---|---|
| `src/duco_ros_driver/include/duco_ros_driver/InterlockPersistence.hpp` | 用尽量原子、可落盘的文件标记记录“联锁仍生效”，使崩溃/重启后不会静默恢复；负责目录创建、写、删、读与错误文本 | `class InterlockPersistence`、`defaultPath()`（`$XDG_STATE_HOME/duco/dual_trajectory_interlock.state` 或 `$HOME/.local/state/duco/...`）、`prepare()`、`write()`、`remove()`、`read()`、`exists()`、`fileExists()`、`directoryOf()`、`makeDirectories()` | POSIX（`fcntl`/`stat`/`unistd`）+ STL |
| `src/duco_ros_driver/test/test_interlock_persistence.cpp` | 15 个用例 | — | gtest |

### 8.2.7 节点级集成与任务生命周期的自动化验证

| 路径 | 职责 | 关键类/函数 | 依赖 |
|---|---|---|---|
| `src/duco_ros_driver/test/test_dual_trajectory_action.cpp` | 36 个集成用例：真实构造节点 + 模拟孪生/实体通道，覆盖目标被拒、孪生失败、实体未确认、取消超时、故障落盘失败阻塞实体运动、`clear_fault` 恢复等 | gtest 夹具 + 话题桩 | gtest、rclcpp、control_msgs |

### 8.2.8 仿真 / 实机 Launch 配置

| 路径 | 职责 | 关键类/函数 | 依赖 |
|---|---|---|---|
| `src/duco_gcr5_910_moveit_config/launch/digital_twin_real.launch.py` | 唯一“孪生 + 实体联动”入口：启动 Gazebo、`ros2_control`、MoveIt、孪生 Action Server，并按参数决定是否启用实体通道；含启动期参数校验 | `load_yaml_file()`、`_is_true()`、`validate_real_robot_settings()`（OpaqueFunction）、`generate_launch_description()` | `launch`、`launch_ros`、`ros_gz_sim`、`moveit_configs_utils`、本包 config |
| `src/duco_gcr5_910_moveit_config/launch/gazebo.launch.py` | 纯仿真启动（不含实体通道），用于数字孪生预演与回归 | `generate_launch_description()` | `launch_ros`、`ros_gz_sim`、`ros2_control` |

### 8.2.9 轨迹发送工具（实机/孪生均可用）

| 路径 | 职责 | 关键类/函数 | 依赖 |
|---|---|---|---|
| `src/duco_gcr5_910_moveit_config/scripts/send_joint_waypoints.py` | 运维工具：读取 YAML 路径点，发送**一个** `FollowJointTrajectory` 目标，并用退出码如实报告结果。**不重试、不自动发送下一条轨迹**；取消/中止一律视为失败 | `main()`、`load_trajectory()`、`class JointWaypointSender`（`send()`，可选注入 `action_client` 仅供测试）、`class TrajectoryOutcomeError`（`describe()`）、`check_outcome()`、`describe_status()`、`describe_error_code()`、`failure_message()`、常量 `SUCCESS_EXIT_CODE=0` / `FAILURE_EXIT_CODE=1` | `rclpy`、`rclpy.action`、`action_msgs.msg.GoalStatus`、`control_msgs.action.FollowJointTrajectory`、`trajectory_msgs`、`PyYAML` |
| `src/duco_gcr5_910_moveit_config/test/test_send_joint_waypoints.py` | 63 个用例，分四组：纯判定逻辑 / 模拟 Action Client / 假发送器的 `main()` 控制流与 rclpy 生命周期 / **进程内模拟 Action Server 的端到端**（真实 ROS 2 action 通信，含用真实 `CancelGoal` 服务请求制造 CANCELED） | `_load_script_under_test()` 等 | `pytest`、`rclpy`、`action_msgs`、`control_msgs` |

### 8.2.10 数字孪生端到端验证脚本

| 路径 | 职责 | 关键函数/结构 | 依赖 |
|---|---|---|---|
| `scripts/dual_twin_verification/verify_all.sh` | 端到端入口：Phase A（Gazebo+MoveIt 孪生）/ B（独立节点实例对孪生）/ C（实体通道未被误触发），任一阶段失败即非零退出 | Shell | `env.sh`、`phase_*` |
| `scripts/dual_twin_verification/env.sh` | 统一环境（ROS 源、PATH、日志目录） | Shell | — |
| `scripts/dual_twin_verification/twin_common.py` | 共享工具：结果汇报器、Future 等待、孪生动作服务器桩 `class TwinHarness(Node)`、`add_scripted_*` 等 | `class Reporter`、`wait_future()`、`class TwinHarness` | `rclpy`、`control_msgs` |
| `scripts/dual_twin_verification/phase_a_check.py` | Phase A：孪生 Action Server 可用、`/joint_states` 可用、下发后到达终止状态且关节实际运动 | — | 同上 |
| `scripts/dual_twin_verification/phase_b_checks.py` | Phase B：22 项节点级检查（拒绝、失败、取消、超时、联锁、恢复等） | — | 同上 |
| `scripts/dual_twin_verification/phase_c_check.py` | Phase C：8 项“实体通道未被误触发”检查 | — | 同上 |
| `scripts/dual_twin_verification/phase_a_moveit_gazebo.sh` / `phase_b_node.sh` / `stop_all.sh` | 分阶段启动与清理（清理脚本对 `move_group` 使用 SIGKILL） | Shell | — |

### 8.2.11 PLC 桥接

| 路径 | 职责 | 关键函数 | 依赖 |
|---|---|---|---|
| `src/plc_s7_bridge/plc_s7_bridge/plc_s7_bridge.py` | 通过 `python-snap7` 读写西门子 S7-1214C 的 DB 位，作为 ROS 2 与 PLC 的联动桥 | 见文件内 `main()`/回调 | `rclpy`、`snap7`（外部库） |
| `src/plc_modbus_bridge/plc_modbus_bridge/plc_modbus_bridge.py` | 以 Modbus TCP 服务器方式提供同一组 PLC 位控制 | 同上 | `rclpy`、`pymodbus`（外部库） |

上述两个包自声明许可为 `BSD-3-Clause`（见 §8.5）。

---

## 8.3 功能边界（能力与证据强度）

**图例**：✅ 已实现 ｜ 🧪 有自动化测试 ｜ 🎮 已在数字孪生（Gazebo）中跑通 ｜ 🤖 未在实体机械臂上验证 ｜ 🏭 依赖厂商 SDK ｜ ⚠️ 设备状态本身不确定

| 功能 | 状态 | 说明与证据 |
|---|---|---|
| 接收上位机 `FollowJointTrajectory` 目标并串行执行 | ✅🧪🎮 | 节点 `DualTrajectoryAction`；孪生 Phase A/B 实跑通过 |
| 轨迹合法性校验（关节名、点数、有限值、时间单调、软限位） | ✅🧪 | `TrajectoryValidation.hpp`，26 个 gtest 全通过 |
| 先孪生后实体（孪生成功才下发实体） | ✅🧪🎮 | 孪生端失败即不下发实体，Phase B 覆盖 |
| 实体通道默认关闭 | ✅🧪 | `publish_real` 默认 `false`；launch 中 `use_real_robot` 默认 `false` |
| 实体反馈校验（序列号 + 时效 + 容差 + 速度） | ✅🧪 | `RealFeedbackGate.hpp`，39 个 gtest 全通过 |
| 实体运动异常联锁（未确认结束/不确定/故障 → 阻塞后续实体运动） | ✅🧪 | `RealMotionInterlock.hpp`，14 个 gtest；节点级 36 个集成用例 |
| 联锁落盘与重启后恢复 | ✅🧪 | `InterlockPersistence.hpp`，15 个 gtest；写失败时阻塞实体运动（`ARuntimeWriteFailureBlocksRealMotion`） |
| 人工恢复 `clear_fault` 服务 | ✅🧪🎮 | 需显式调用；默认不自动恢复（`fault_recovery_enabled=false`） |
| 轨迹发送工具如实报告成功/失败 | ✅🧪 | 本轮修复；63 个 pytest 全通过（含端到端组） |
| 轨迹发送工具自动重试 / 自动下发下一条 | ❌ 刻意不实现 | 失败后由人工决策；避免把“未知结果”变成“自动继续运动” |
| 数字孪生端到端回归 | ✅🎮 | `verify_all.sh` 退出码 0（A 5/5、B 22/22、C 8/8） |
| **实体机械臂上的真实运动验证** | 🤖 **未执行** | 受现场安全负责人书面授权限制；本轮未做任何真实运动 |
| 联锁触发后的**硬件级**停止确认 | ⚠️ **无法由软件证明** | 厂商未发布停止确认接口（`controller_connected_flag_`、`robotmoving()` 均未发布），停止沿用“下发空 `JointTrajectory`”约定。**取消超时意味着“执行结果未知”，不等于“机械臂已停下”** |
| 轨迹被“取消/中止”时的真实关节位置 | ⚠️ 不确定 | 只能靠实体反馈话题；若反馈中断，软件无法判断实际位置 |
| 上位机（客户 HMI）侧 Action Client 行为 | 🏭 **不在本仓库内** | 是否读取终止状态/`error_code`/`error_string`、是否会自动重发，**未经核对**，必须由上位机负责人确认 |
| 厂商 SDK 通信与控制器状态 | 🏭⚠️ | 依赖 `libDucoCobotAPI.a`（厂商提供，源码不在本仓库）；`GET /`、`PUT /` 之外的行为无法验证 |
| PLC 联动 | 🏭 | 依赖外部 `snap7`/`pymodbus` 与真实 PLC；本轮无 PLC 硬件，未执行 |

### 8.3.1 必须保留的安全表述

1. 本项目的联锁**是软件联锁**，它约束的是“本节点是否继续下发实体轨迹”，
   **不是**硬件安全功能，**不能**代替急停、安全门锁、安全继电器等安全措施。
2. 退出码 0 只表示“Action Server 确认该轨迹执行完毕”，**不表示**任何硬件级完成确认；
   非零退出码也不代表机械臂已停止。
3. 任何“失败后自动重发/自动继续”都会把不确定状态放大，因此本项目刻意不实现。

---

## 8.4 关键接口（Action / Topic / Service / Launch）

### 8.4.1 Action

| 名称 | 默认值 | 方向 | 类型 | 说明 |
|---|---|---|---|---|
| `front_action_name` | `arm_1_controller/follow_joint_trajectory`（相对名） | **服务端**（本节点对外提供） | `control_msgs/action/FollowJointTrajectory` | 上位机/操作者下发的入口；一次只接受一个目标 |
| `sim_action_name` | `sim_arm_1_controller/follow_joint_trajectory`（相对名） | **客户端**（本节点调用） | 同上 | 指向 Gazebo `joint_trajectory_controller` |

> 命名细节：节点参数使用**相对**动作名（解析后位于根命名空间之下）；
> `send_joint_waypoints.py` 的 `--action` 默认值是写死的**绝对**名
> `/arm_1_controller/follow_joint_trajectory`。两者在根命名空间下指向同一个动作，
> 但若把节点放到子命名空间下运行，必须显式给 `--action` 传对应的绝对名。

### 8.4.2 Topic

| 名称 | 默认值 | 方向 | 类型 | QoS/深度 | 说明 |
|---|---|---|---|---|---|
| `real_command_topic` | `joint_path_command` | 发布 | `trajectory_msgs/msg/JointTrajectory` | 10 | 实体通道命令；**只有 `publish_real:=true` 时才真正发布**；停止约定=空轨迹 |
| `real_feedback_topic` | `real_feedback_states` | 订阅 | `control_msgs/action/FollowJointTrajectory_Feedback` | 100 | 实体反馈（需外部提供） |
| `real_status_topic` | `/duco_cobot/robot_state` | 订阅 | `duco_msg/msg/DucoRobotState` | 30 | 实体机器人状态；`healthy` 参与门控 |
| `real_motion_state_topic` | `dual_trajectory/real_motion_state` | 发布 | `std_msgs/msg/String` | 10 | **仅诊断**，任何控制决策都不消费它；置空则不创建 |
| `real_command_topic` 的消费方 | — | — | — | — | 厂商 `DucoDriver` 订阅 `joint_path_command`（深度 2000）后经 SDK 下发 |

### 8.4.3 Service

| 名称 | 默认值 | 类型 | 说明 |
|---|---|---|---|
| `clear_fault_service` | `clear_fault` | `std_srvs/srv/Trigger` | 人工清除联锁/故障标记；是否可用受 `fault_recovery_enabled`（默认 `false`）约束 |

### 8.4.4 节点参数（`DualTrajectoryAction`，默认值即安全默认）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `publish_real` | `false` | **实体通道总开关，必须显式打开** |
| `wait_for_real` | `false` | 是否等待实体通道完成反馈 |
| `publish_real_stop_request` | `true` | 需要停止时是否发布空轨迹 |
| `real_goal_tolerance` / `real_velocity_tolerance` | `0.02` / `0.05` | 门控容差 |
| `real_feedback_max_age` / `real_status_max_age` | `1.0` / `1.0` | 反馈/状态最大允许时延（秒） |
| `real_require_velocity` | `true` | 是否强制要求速度反馈 |
| `fault_recovery_enabled` | `false` | 是否允许 `clear_fault` 恢复 |
| `latch_on_real_fault` | `true` | 实体故障是否置位锁存 |
| `real_unhealthy_robot_states` | `[]` | 额外视为不健康的状态码 |
| `min_real_trajectory_points` | `2` | 实体轨迹最少点数 |
| `fault_latch_file` | `""`（空=用 `defaultPath()`） | 联锁标记文件 |
| `real_persistence_required` | `true` | 落盘失败时是否阻塞实体运动 |
| `real_execution_timeout` / `sim_execution_timeout` | `300.0` / `300.0` | 执行超时（秒） |
| `wait_for_sim_seconds` / `acceptance_timeout` | `5.0` / = 前者 | 等待孪生 Action Server / 目标被接受 |
| `cancel_timeout` | `5.0` | 取消等待上限；超时按“结果未知”处理 |
| `sim_time_scale` | `1.0` | 孪生轨迹时间缩放 |
| `enforce_expected_joint_names` / `expected_joint_names` | `true` / 6 个关节名（由 launch 注入） | 关节名校验 |
| `joint_position_min` / `joint_position_max` | `[]`（由 launch 注入） | 软限位 |

### 8.4.5 Launch 参数（`digital_twin_real.launch.py`，与上一版兼容性）

| 参数 | 默认值 | 兼容性变化 |
|---|---|---|
| `use_real_robot` | `false` | **必须显式置 `true`** 才会启用实体通道 |
| `server_host_1` | 无默认（空） | **行为变更**：`use_real_robot:=true` 时必须显式给定；不再使用历史硬编码 IP |
| `wait_for_real` | `false` | 与 `use_real_robot` 一致性在启动期校验，矛盾即报错退出 |
| `use_real_control_services` / `arm_num` / `arm_dof` | `true` / `1` / `6` | 未变 |
| `use_rviz` / `world` / `gz_args` / `initial_positions_file` | `true` / 包内 SDF / `-r -v 3` / 包内 YAML | 未变 |
| 节点参数改名 | `real_feedback_timeout`→`real_feedback_max_age`，`real_status_timeout`→`real_status_max_age` | **部署需同步改名**（上一轮已记录） |
| `fault_latch_file` | 空 → 用户态目录 | 部署需确认该目录**可写**，否则实体运动会被阻塞 |

---

## 8.5 第三方依赖与代码归属边界

### 8.5.1 运行期第三方依赖

| 依赖 | 版本/来源 | 用途 | 归属 |
|---|---|---|---|
| ROS 2 Jazzy (`rclcpp`, `rclpy`, `rclcpp_action`) | 系统安装 | 节点/动作/通信 | 第三方（Apache-2.0） |
| MoveIt 2.12.3 | 系统安装 | 运动规划、`MoveGroup`、轨迹执行 | 第三方（BSD-3-Clause） |
| `ros2_control` + `joint_trajectory_controller` | 系统安装 | 孪生控制器 | 第三方 |
| Gazebo（`ros_gz_sim`, `gz_*`） | 系统安装 | 孪生仿真 | 第三方 |
| `control_msgs` / `action_msgs` / `trajectory_msgs` / `std_srvs` | 系统安装 | 接口定义 | 第三方 |
| `PyYAML`, `pytest` | 系统包（`python3-yaml` 6.0.1、`python3-pytest` 7.4.4） | 工具/测试 | 第三方 |
| `snap7` / `pymodbus` | Python 包 | PLC 桥接 | 第三方 |
| `libDucoCobotAPI.a`（`src/duco_ros_driver/lib/`） | 厂商二进制库，**无源码** | 与 DUCO 控制器通信 | **厂商资产**；未修改 |

### 8.5.2 仓库内代码归属（判定依据：文件头版权声明、`package.xml` 许可字段、目录来源）

| 路径 | 判定 | 依据 | 需人工确认项 |
|---|---|---|---|
| `src/duco_ros_driver/include/duco_ros_driver/{DualTrajectoryAction,TrajectoryValidation,RealFeedbackGate,RealMotionInterlock,InterlockPersistence}.hpp`、`src/DualTrajectoryAction.cpp`、`test/test_*.cpp` | 本项目自研（依据为内容与提交历史；**著作权归属仍以项目负责人确认结论为准**） | 无第三方版权头 | 与单位/项目方的著作权归属结论 |
| `src/duco_ros_driver/src/{DucoDriver,DucoDriverNode,DucoRobotControl,DucoRobotStatus,DucoRobotStatusNode,RobotNode,DucoTrajectoryAction}.cpp`、`include/duco_ros_driver/{joint_relay_handler,param_utils,utils,utils_}.h` | **厂商/上游派生**，未修改 | 文件头含 SIASUN Robotics (c) 2017-2018、Southwest Research Institute (c) 2012 BSD 等声明 | 再分发许可与修改许可 |
| `src/duco_msg/`（`.msg`/`.srv` 定义） | 厂商接口定义 | `package.xml` 为 `TODO: License declaration` | 许可声明缺失，需补 |
| `src/duco_ros_driver/package.xml` | 许可字段为 `TODO: License declaration` | 同上 | 需补正式许可声明 |
| `src/duco_support/`（URDF/SDF/meshes） | 厂商资源 | 目录内 `LICENSE` 为 **Apache-2.0**，而 `package.xml` 写 `BSD`（**两者不一致**） | 再分发权、许可一致性 |
| `src/duco_gcr5_910_moveit_config/`（启动/配置/网格） | 生成包 + 厂商资源混装 | `package.xml` 写 `BSD`；含厂商 URDF/SDF/mesh | 厂商资源再分发权 |
| `src/plc_s7_bridge/`、`src/plc_modbus_bridge/` | 自研 | `package.xml` 声明 `BSD-3-Clause` | 与单位的一致结论 |
| `scripts/dual_twin_verification/` | 自研 | 无第三方声明 | — |

### 8.5.3 处置原则（后续开发者必须遵守）

1. **不得删除或改写第三方版权声明**（SIASUN / Southwest Research Institute / 厂商 SDF/URDF 内的声明）。
2. 任何“自研”判定只是工程结论，**不构成法律结论**；上述“需人工确认项”在补全前应标记为**未确认**。
3. 未确认再分发权的资源（厂商 mesh/URDF/SDF）不应对外发布二进制包。
4. 许可字段为 `TODO` 的包在正式发布前必须补齐。

---

## 8.6 验证状态（实际执行过的命令与结果）

环境：Ubuntu 24.04 + ROS 2 Jazzy；命令均在 `source /opt/ros/jazzy/setup.bash` 后执行。

### 8.6.1 编译

| 命令 | 结果 |
|---|---|
| `colcon build`（全工作区，6 个包） | ✅ 全部成功 |
| `rm -rf build/duco_ros_driver install/duco_ros_driver && colcon build --packages-select duco_ros_driver --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3` | ✅ 冷配置 + 全量重编译成功（14.9s）；输出仅有厂商派生源码既有的 `-Wsign-compare` 警告 |
| `colcon build --packages-select duco_gcr5_910_moveit_config` | ✅ 成功；CMake 输出 `-- send_joint_waypoints.py tests run with /usr/bin/python3` |

> **环境注意（可复现）**：当前机器 `python3` 被 miniconda 3.13 遮挡，`FindPython3` 会选中它，
> 而它没有 `catkin_pkg`，导致**冷配置**报错 `No module named 'catkin_pkg'`。
> 现有各包的缓存均使用 `/usr/bin/python3`。因此冷配置请显式指定解释器
> （`-DPython3_EXECUTABLE=/usr/bin/python3`），或先 `conda deactivate`。
> 这与本轮代码改动无关，属于既有环境问题。

### 8.6.2 单元与集成测试

| 命令 | 结果 |
|---|---|
| `colcon test --packages-select duco_ros_driver` | ✅ 5 个 gtest 套件全部通过：`test_dual_trajectory_action` 36、`test_real_feedback_gate` 39、`test_trajectory_validation` 26、`test_interlock_persistence` 15、`test_real_motion_interlock` 14 = **130/130 通过，0 失败，0 跳过** |
| `colcon test --packages-select duco_gcr5_910_moveit_config` | ✅ `test_send_joint_waypoints` 通过（**63/63**） |
| `/usr/bin/python3 -m pytest src/duco_gcr5_910_moveit_config/test/test_send_joint_waypoints.py -q`（连跑 3 次） | ✅ 63 passed，每次约 3.0s，无抖动 |
| `colcon test-result --verbose`（全工作区汇总） | 271 个测试条目、**0 errors**、37 failures、28 skipped |

**37 个 failures 的完整归属（全部为历史遗留，与本次改动无关）**：

| 来源 | 数量 | 说明 |
|---|---|---|
| `uncrustify` | 24 | `duco_ros_driver` 内**既有**未格式化文件（含厂商派生文件与早期测试）；本轮未新增/修改任何 C/C++ 文件，输入逐字节相同 |
| `lint_cmake` | 11 | `duco_ros_driver/CMakeLists.txt` 既有风格问题（行列长度、括号内多余空格），行号 76/79/81/83/85/87 |
| CTest 聚合条目 | 2 | 上述两个 ctest 包装测试本身的“失败”状态 |
| `cppcheck` | 28 skipped | Jazzy 自带 cppcheck 2.13.0 性能问题（ament 主动跳过），基线一致 |

> 表述纪律：**不应**把“核心功能测试全通过”表述为“整个测试套件零失败”。上表 37 项失败全部是既有的格式/风格类问题。

### 8.6.3 数字孪生端到端

| 命令 | 结果 |
|---|---|
| `bash scripts/dual_twin_verification/verify_all.sh` | ✅ 退出码 **0**；Phase A **5/5**、Phase B **22/22**、Phase C **8/8**；无残留仿真进程 |

### 8.6.4 Python 语法检查

| 命令 | 结果 |
|---|---|
| `/usr/bin/python3 -m py_compile <仓库内 22 个 .py>` | ✅ 22/22 通过 |

### 8.6.5 Lint：新增代码是否引入新问题

以可证明的方式回答，而不是重新跑一遍主观比较：

1. **本轮改动的文件集合**（`git diff --name-only 1faf8f7..454bb5a`）只含 `.py` / `CMakeLists.txt` / `package.xml` / `.md`，
   **没有任何 `.cpp/.hpp/.h`** → `uncrustify` 与 `cppcheck` 的输入逐字节不变 → 结果必然与上一轮相同（实测：`uncrustify` 28 项中 24 项失败、`cppcheck` 28 项全部 skipped，与基线一致）。
2. **`lint_cmake` 只检查 `duco_ros_driver`**；本轮改动的 `CMakeLists.txt` 属于 `duco_gcr5_910_moveit_config`，该包**未启用任何 lint 测试** → 不进入 CI lint 面。
3. **Python lint（信息性，非 CI）**：该仓库没有为任何包启用 `ament_flake8`。用 ROS 自带的 `ament_flake8.ini` 对改动前后文件做对照：
   - 修改前 `send_joint_waypoints.py`：Q000 30、E501 3、I100 1
   - 修改后：Q000 70（随代码量增加，**沿用文件既有双引号约定**）、E501 **仍为 3 且是同样的 3 行**（`>99` 字符的行在修改前就存在、文本未变）、I100 1
   - 结论：**未引入新类别的 lint 问题**；Q000 属该文件与仓库既有风格，E501 为历史行。
4. `xmllint` 校验修改后的 `package.xml`：✅ 格式良好。

### 8.6.6 本轮**未执行**的验证（不得视为通过）

| 项目 | 原因 |
|---|---|
| 实体机械臂真实运动验证 | 现场安全负责人书面授权 + 单独许可尚未取得；本轮未做任何真实运动 |
| 真实 PLC 联动（`snap7`/`pymodbus`） | 无 PLC 硬件环境 |
| 上位机/客户 HMI 的 Action Client 行为 | 不在本仓库，需上游确认 |
| 硬件级停止确认 | 厂商未提供接口，**无法用软件验证** |
| 旧的双临界区落盘并发竞态 | 无法确定性复现（临时回退修复后压力测试仍通过）；原子性由“单一临界区”结构保证，唯一可确定性判据是 `ARuntimeWriteFailureBlocksRealMotion` |

### 8.6.7 本轮 §5 复核清单的复核结论（只读复核，未重写）

| 复核项 | 结论 | 证据 |
|---|---|---|
| 轨迹处理（解析/校验/时间/限位） | 通过 | `TrajectoryValidation.hpp` + 26 用例 |
| Action 生命周期（接受/执行/取消/超时/终止） | 通过 | `DualTrajectoryAction.hpp` 状态机 + `test_dual_trajectory_action.cpp` 36 用例 + 孪生 Phase B 22 项 |
| 联锁（触发/锁存/恢复/持久化） | 通过（软件联锁，非硬件安全功能） | `RealMotionInterlock.hpp` / `InterlockPersistence.hpp` + 29 用例 |
| 反馈校验（序列号/时效/容差/健康位） | 通过 | `RealFeedbackGate.hpp` + 39 用例 |
| 安全默认（实体通道默认关闭、无自动重试/自动续发） | 通过 | `publish_real=false`、`use_real_robot=false`、`fault_recovery_enabled=false`；工具一次只发一个目标（`test_main_does_not_retry_and_does_not_send_a_second_goal`） |

---

## 8.7 最终文件清单（相对路径 / 类型 / 行数 / SHA-256）

> 取自冻结提交 `454bb5a` 的文件内容；行数为 `wc -l`，SHA-256 为完整 64 位十六进制。

### A. 自研核心：`duco_ros_driver`

| 相对路径 | 类型 | 行数 | SHA-256 |
|---|---|---|---|
| `src/duco_ros_driver/include/duco_ros_driver/DualTrajectoryAction.hpp` | C++ 头 | 1838 | `73d436009c79355ab42af5eb12d5894de285122b24855748c3622bf3b361f137` |
| `src/duco_ros_driver/include/duco_ros_driver/TrajectoryValidation.hpp` | C++ 头 | 226 | `6bc8ee00a64dca36b38f1a3d891c141276a67174fec38e362fbe71e1debf2c0e` |
| `src/duco_ros_driver/include/duco_ros_driver/RealFeedbackGate.hpp` | C++ 头 | 253 | `4fe5ac28294b7f53a92d8f67b582c43bcf397328bc17ae372681f11025c527f9` |
| `src/duco_ros_driver/include/duco_ros_driver/RealMotionInterlock.hpp` | C++ 头 | 199 | `2c8c19b4f00e1abf0c229556ef0467da51fbc45a9bb5c3620ebccbb475cb3cc0` |
| `src/duco_ros_driver/include/duco_ros_driver/InterlockPersistence.hpp` | C++ 头 | 350 | `185e2269c49c00e48a3cfe975dde7e49fc8835f794ba1055a67385ffb29b9978` |
| `src/duco_ros_driver/src/DualTrajectoryAction.cpp` | C++ 源 | 28 | `ee846051cb07c4ab64f24ad43f6335febe3e2a2a5af4288191453a357cb8a84e` |
| `src/duco_ros_driver/test/test_trajectory_validation.cpp` | C++ 测试 | 386 | `f9d816342c6a6391c7b5ae94fe47b11af946d199336af398492251c411efe187` |
| `src/duco_ros_driver/test/test_real_feedback_gate.cpp` | C++ 测试 | 473 | `f4c3400f55140b19c803bbeadabcc2ed8e5eb5c36138da201460f32199e9a4b1` |
| `src/duco_ros_driver/test/test_real_motion_interlock.cpp` | C++ 测试 | 165 | `238852a6a1e79f73bf21e338bcc4672e6c9e942bfc627c98a7414b107fc30de9` |
| `src/duco_ros_driver/test/test_interlock_persistence.cpp` | C++ 测试 | 283 | `48439dcee1ad7893b59298895843d7569f7bcb8f3134055734e0d07492170b95` |
| `src/duco_ros_driver/test/test_dual_trajectory_action.cpp` | C++ 测试 | 1376 | `44e415fc935f20c248e600ad6b6df5f39b19b116a41b97613a616a6dc8b36716` |
| `src/duco_ros_driver/CMakeLists.txt` | CMake | 93 | `7f3b6f5cb4ca03ba5c5a593045c22a03563806ebcdf0ba8134774aa2fea0a1c0` |
| `src/duco_ros_driver/package.xml` | XML | 42 | `c282f4dc4704f8ba80a48f8b18a3ee74a25440e645ca0dc662c2c58d8371a811` |

### B. 自研核心：`duco_gcr5_910_moveit_config`（Launch / 工具 / 测试）

| 相对路径 | 类型 | 行数 | SHA-256 |
|---|---|---|---|
| `src/duco_gcr5_910_moveit_config/launch/digital_twin_real.launch.py` | Python | 363 | `929c3d1b2d0fea78f84f8de7ccdf5f2287efdb270bf39d520a03097461ec1f6e` |
| `src/duco_gcr5_910_moveit_config/launch/gazebo.launch.py` | Python | 155 | `fe8730384cbce16b3ed63dbcfe18f7f89c35cc646023670064b22ad338de592a` |
| `src/duco_gcr5_910_moveit_config/scripts/send_joint_waypoints.py` | Python | 305 | `955cd7fd591092b98aaa7c7ab22b535daa3f13aa2904ebb93d6c36fea3e877d2` |
| `src/duco_gcr5_910_moveit_config/test/test_send_joint_waypoints.py` | Python 测试 | 912 | `05f68268a016042f4ac68e115d83ab89b8a24151abcad1c3a214f8b02612d518` |
| `src/duco_gcr5_910_moveit_config/config/example_joint_waypoints.yaml` | YAML | 22 | `6c27d109da0a1b776a003d4a717f7f87954e413e919a32604ea8dbbf4df2f299` |
| `src/duco_gcr5_910_moveit_config/CMakeLists.txt` | CMake | 58 | `270a7d4a985853a03635ae17a2b6337089ce365c7ec0f249a11bc4904e26d32a` |
| `src/duco_gcr5_910_moveit_config/package.xml` | XML | 63 | `0a29f7873c4cd80e745fea1e0b96aea66b621fae1e16799fab9888235a4cc360` |

### C. 自研核心：数字孪生端到端验证

| 相对路径 | 类型 | 行数 | SHA-256 |
|---|---|---|---|
| `scripts/dual_twin_verification/verify_all.sh` | Shell | 80 | `4ecdacf2dd4222c674f82d6096b99d0c9998baff01d769e639553454099119eb` |
| `scripts/dual_twin_verification/env.sh` | Shell | 38 | `63f8a653f9a77bfa5169df56d76ef609b010af30e07dbcb154275b61fda31f43` |
| `scripts/dual_twin_verification/twin_common.py` | Python | 235 | `b1d89697956bc256614202ac9da9b6cb20f072cf4e8be2b51e1d82626e90b1a0` |
| `scripts/dual_twin_verification/phase_a_check.py` | Python | 84 | `f609c25f64cc2673a938eda2810179b4cf1d30eb23cf10ffbb42e6046d203685` |
| `scripts/dual_twin_verification/phase_b_checks.py` | Python | 166 | `dfe7e5505d2035f63229aebee5f9d8a80810d3fb0859ce8a67af8eac01cdd8df` |
| `scripts/dual_twin_verification/phase_c_check.py` | Python | 81 | `33a8b59e761862949511dd53d525608c30269f198f77b818d53d3f9574c1b039` |
| `scripts/dual_twin_verification/phase_a_moveit_gazebo.sh` | Shell | 17 | `4b43eb9f3d82fdb02d05edc02a8ddc23b41f31bfdc7082e0f065b856b325c9a6` |
| `scripts/dual_twin_verification/phase_b_node.sh` | Shell | 41 | `92f104eed6eae9ebb05743624809053018818fcdee5820f78789e7df1a557b07` |
| `scripts/dual_twin_verification/stop_all.sh` | Shell | 59 | `52d20df31341d28a6d7335040cccab63fbeccef699f08924446b1fdd03c9cdd1` |

### D. 自研核心：PLC 桥接（自声明 BSD-3-Clause）

| 相对路径 | 类型 | 行数 | SHA-256 |
|---|---|---|---|
| `src/plc_s7_bridge/plc_s7_bridge/plc_s7_bridge.py` | Python | 437 | `58d9f7fea0856afe60d1b9ba83f8ea3ced559455a8c92e70b3c3936e1a999d58` |
| `src/plc_s7_bridge/setup.py` | Python | 24 | `372b1dde9064706f7ec89523ee85737163198e59721522cb2a0e4c525461dbd5` |
| `src/plc_s7_bridge/package.xml` | XML | 19 | `380c914200f38bd6ee0ce3013d594939ead6b7eec44da88a5db1a0beedfd2196` |
| `src/plc_modbus_bridge/plc_modbus_bridge/plc_modbus_bridge.py` | Python | 278 | `da27ae6da3dc9a71842a40141d63e680885dc8580daa1eb051923687b9f1b778` |
| `src/plc_modbus_bridge/setup.py` | Python | 24 | `5a1e24769862f1134d07c60f23d8b4bc1896d6137a27227b3716b245f6e2ece3` |
| `src/plc_modbus_bridge/package.xml` | XML | 17 | `3eab1f4a34abb98e87a5804c29442b4123d316f4b99d354f155904f335488bde` |

### E. 厂商派生 / 厂商资产（**非本项目自研**，仅登记以便追溯；不得删改其版权声明）

| 相对路径 | 类型 | 行数 | SHA-256 |
|---|---|---|---|
| `src/duco_ros_driver/src/DucoDriver.cpp` | C++（厂商派生） | 347 | `9df84a1bfc2a0a1caa91c4f1d462230ad9b3a17e7d47619c0e01ca7b09e005dd` |
| `src/duco_ros_driver/src/DucoDriverNode.cpp` | C++（厂商派生） | 61 | `01677294d730bd72757e117289984868a8337cd49f6ed26c693c54d31f567752` |
| `src/duco_ros_driver/src/DucoRobotControl.cpp` | C++（厂商派生） | 478 | `593c7dcd43a46ba0b8372cf8898314480a18c12d6529f347f0e01a09be51774a` |
| `src/duco_ros_driver/src/DucoRobotStatus.cpp` | C++（厂商派生） | 313 | `232185c0d625c7548d814004f81a6f825d5fe8994d0d7bda179cd4598798e990` |
| `src/duco_ros_driver/src/DucoRobotStatusNode.cpp` | C++（厂商派生） | 63 | `6e69be69e84027eb407d7bc3cf7a38240ed6344a28a268b583443e5db3273f37` |
| `src/duco_ros_driver/src/RobotNode.cpp` | C++（厂商派生） | 34 | `1bf31c6b8ec45f27a8aeb8764b94867e790942e00555392cc5038eccddf0808a` |
| `src/duco_ros_driver/src/DucoTrajectoryAction.cpp` | C++（厂商派生） | 387 | `5901eb81705f9e4f70decb5e3586d9238a6c23562624071ff796b767d167a9f8` |
| `src/duco_ros_driver/include/duco_ros_driver/joint_relay_handler.h` | C++ 头（厂商派生） | 159 | `0c927f210dcc725004e75168d9c2f0b128b07fbd0686cebbb5a5aaed0c327026` |
| `src/duco_ros_driver/include/duco_ros_driver/param_utils.h` | C++ 头（厂商派生） | 80 | `19f42f44b8065f191f2d732cd3de074b6c2ef7faf819c53f6ce7a15683fa356a` |
| `src/duco_ros_driver/include/duco_ros_driver/utils.h` | C++ 头（厂商派生） | 122 | `828104be62f1762feb2729f281caec8da205b20e7277fe964905052f56245192` |
| `src/duco_ros_driver/include/duco_ros_driver/utils_.h` | C++ 头（厂商派生） | 82 | `35e3d4c4264a5948d26da9ff3d83b9b2684ce93381ff6c68645229525c1f7fa1` |
| `src/duco_msg/msg/DucoRobotState.msg` | 接口定义 | 22 | `55dd2b666ec8b4e7da3197830bd73038744c9d6b9dff18934e9ab83a7f5d0c3a` |
| `src/duco_msg/srv/RobotMove.srv` | 接口定义 | 17 | `b30f5105ea3fe4fa0ea12fcfd080d369dcd221d4b7074956229410fa0d09a850` |
| `src/duco_support/package.xml` | XML | 22 | `630e698d0e1f48b302c69acba25f6d5718a923ed411b68c8e46c36efcc371617` |

（`src/duco_support/` 的 URDF/SDF/mesh 与 `src/duco_msg/` 的其余 `.msg/.srv` 未逐条列出，属厂商资产。）

---

## 附录 A：已知限制（交接时必须一并转达）

1. **软件联锁 ≠ 硬件安全功能**（重复强调，见 §8.3.1）。
2. 取消超时后真实执行结果**未知**，只能由人工按流程处理；没有任何厂商接口可以确认“已停止”。
3. 上位机 Action Client 的行为（是否读终止状态、是否自动重发）**未经核对**，必须由上游确认。
4. 落盘并发竞态无法确定性复现，依赖结构性论证（单一临界区）+ `ARuntimeWriteFailureBlocksRealMotion` 判据。
5. `duco_msg` / `duco_ros_driver` 的 `package.xml` 许可仍为 `TODO`；`duco_support` 的 `LICENSE`（Apache-2.0）与 `package.xml`（BSD）不一致；厂商资源的再分发权未确认。
6. 冷配置需要显式指定 `/usr/bin/python3`（conda 遮挡问题，见 §8.6.1）。
7. `uncrustify` 24 项、`lint_cmake` 11 项为**既有**风格问题，未在冻结前做仓库级重排（避免扩大改动面）。

## 附录 B：后续建议（按优先级）

1. 取得授权后做**实体机械臂**分级验证（先单关节小幅度、低速、带外部急停，再逐步放开），并把结果补进本文档。
2. 请上位机负责人确认 §8.3 表中“上位机行为”一行，并把结论写回兼容性文档。
3. 补齐 `package.xml` 许可声明与厂商资源再分发授权。
4. 在**单独一轮**做仓库级格式化（`uncrustify`/`lint_cmake` 清理），先提交格式化、再提交功能，保持可二分。
5. 为运维脚本接入外层调度器时，确认“非零退出码即停止后续下发”，且**不要**实现失败自动重发。

---

*文档结束。本文档只描述冻结提交 `454bb5a` 与其实测结果；任何后续修改都应新建分支并重新记录验证证据。*
