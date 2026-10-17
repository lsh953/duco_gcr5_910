# 第三轮双轨迹安全修复：集成分支与合并准备报告

- 个人仓库：`lsh953/duco_gcr5_910`
- 组织仓库：`HENU-Makerspace/duco_gcr5_910_ros2_ws`（remote 名 `origin`）
- 工作副本：`/home/me/duco_ros2_ws`（本地分支 `main` = `b47d3e2`，仅作历史留存，未参与本次集成）
- 本轮**不合并 main、不强制推送、不改写任何已有提交**

---

## 一、合并准备核对

### 1.1 提交哈希确认

| 对象 | 哈希 | 备注 |
|---|---|---|
| `origin/codex/dual-trajectory-safety-fixes` | `31b2eaa9dbb52531ede5d6215403d5e87b7ac108` | 已审阅版本，**冻结** |
| `lsh953/codex/dual-trajectory-safety-fixes` | `31b2eaa…`（同上） | 与 origin **完全一致** |
| 本地 `codex/dual-trajectory-safety-fixes` | `31b2eaa…`（同上） | 三处一致 |
| `lsh953/main`（个人仓库 main 基准） | `7e7c34ba398305081b80373b07d5cec0b4bc6f83` | 本轮集成的上游基准 |
| `origin/main` | `d12a65ff433d5bdd72040f67aaf3578a79e4508b` | 组织仓库 main |

已审阅分支的 4 个提交（保持原哈希，未改写）：

```
31b2eaa Break the colcon test result down by linter in the test report
9a6ac8d Correct the wording and the quoted logs of two test claims
2608934 Report the third round of dual trajectory safety fixes
f5b2c42 Make the cancel, interlock durability and task finalization honest
```

### 1.2 分支谱系（是否会发生覆盖）

```
d12a65f  origin/main
   └── … 12 个提交 …               （前两轮可靠性/安全修复）
        7e7c34b  lsh953/main      ← 本轮基准
           └── 4 个提交 ──► 31b2eaa   codex/dual-trajectory-safety-fixes（已审阅）
                            └── 1ffdbca + 文档提交   integration/round3-safety-fixes（本次）
```

| 检查 | 结果 |
|---|---|
| `git merge-base origin/main lsh953/main` | `d12a65f` = `origin/main` 本身 |
| `origin/main` 领先 `lsh953/main` | **0 个提交** |
| `lsh953/main` 领先 `origin/main` | **12 个提交** |
| 是否存在分叉 | **否**（`origin/main` 是 `lsh953/main` 的祖先） |
| 覆盖已有功能的风险 | **无**：两个仓库不是分叉关系，不存在"各自前进"导致的互相覆盖 |

### 1.3 源文件差异比较

**（a）`origin/main` → `lsh953/main`（前两轮已存在，35 个文件，+7104 / −667）**

全部为新增或修改，**没有任何删除**（`git diff --diff-filter=D` 为空）：

```
A  DIGITAL_TWIN_GUIDE.md
M  README.md
A  docs/DUAL_TRAJECTORY_RELIABILITY_REPORT.md
A  docs/DUAL_TRAJECTORY_SAFETY_FIXES_REPORT.md
A  docs/DUAL_TRAJECTORY_SAFETY_FIXES_TEST_REPORT.md
M  docs/PLC_S7_SUCTION_GUIDE.md
A  scripts/dual_twin_verification/{README.md,env.sh,phase_a_check.py,
     phase_a_moveit_gazebo.sh,phase_b_checks.py,phase_b_node.sh,
     phase_c_check.py,stop_all.sh,twin_common.py,verify_all.sh}
M  src/duco_gcr5_910_moveit_config/launch/{digital_twin_real.launch.py,gazebo.launch.py}
M  src/duco_gcr5_910_moveit_config/scripts/send_joint_waypoints.py
M  src/duco_ros_driver/CMakeLists.txt
A  src/duco_ros_driver/include/duco_ros_driver/DualTrajectoryAction.hpp
M  src/duco_ros_driver/include/duco_ros_driver/DucoRobotStatus.h
A  src/duco_ros_driver/include/duco_ros_driver/{RealFeedbackGate.hpp,
     RealMotionInterlock.hpp,TrajectoryValidation.hpp}
M  src/duco_ros_driver/package.xml
M  src/duco_ros_driver/src/{DualTrajectoryAction.cpp,DucoDriver.cpp,
     DucoRobotStatus.cpp,DucoTrajectoryAction.cpp}
A  src/duco_ros_driver/test/{test_dual_trajectory_action.cpp,
     test_real_feedback_gate.cpp,test_real_motion_interlock.cpp,
     test_trajectory_validation.cpp}
M  src/plc_s7_bridge/plc_s7_bridge/plc_s7_bridge.py
```

**（b）`lsh953/main` → `31b2eaa`（本轮，10 个文件，+1755 / −94）**

```
A  docs/DUAL_TRAJECTORY_SAFETY_FIXES_ROUND3_REPORT.md
A  docs/DUAL_TRAJECTORY_SAFETY_FIXES_ROUND3_TEST_REPORT.md
M  scripts/dual_twin_verification/phase_b_checks.py
M  scripts/dual_twin_verification/phase_b_node.sh
M  src/duco_gcr5_910_moveit_config/launch/digital_twin_real.launch.py
M  src/duco_ros_driver/CMakeLists.txt
M  src/duco_ros_driver/include/duco_ros_driver/DualTrajectoryAction.hpp
A  src/duco_ros_driver/include/duco_ros_driver/InterlockPersistence.hpp
M  src/duco_ros_driver/test/test_dual_trajectory_action.cpp
A  src/duco_ros_driver/test/test_interlock_persistence.cpp
```

**结论**：本轮改动**不触碰** PLC 桥、MoveIt 配置（除 launch 的两个参数）、`duco_msg`、
`duco_support` 等既有模块；对 `DucoDriver`、`DucoTrajectoryAction` 等厂商驱动文件**零改动**。

### 1.4 集成分支

| 项 | 值 |
|---|---|
| 分支名 | `integration/round3-safety-fixes` |
| 创建方式 | 从已审阅的 `31b2eaa` 拉出，**已审阅的 4 个提交原样保留** |
| 追加提交 | `1ffdbca`（Result.error_string ＋ lint 中性化），以及本文档所在提交 |
| 位置 | 个人仓库 `lsh953`（按指示，**只推个人仓库**；`origin` 保持只有已审阅分支） |
| 不做的动作 | 不合 `main`、不 `push --force`、不改写历史、不打 tag、不发布 Release |

---

## 二、厂商 SDK 与第三方资源再分发许可检查

### 2.1 本轮新增内容是否含第三方资源

**不含。** 本轮 10 个改动文件全部为自研：

| 新增文件 | 作者版权头 | 第三方内容 |
|---|---|---|
| `include/duco_ros_driver/InterlockPersistence.hpp` | 无第三方头 | 无（纯 C++/POSIX 自研） |
| `test/test_interlock_persistence.cpp` | 无第三方头 | 无 |
| 两份 round3 报告 ＋ 本文档 ＋ 另两份本轮文档 | 自研 | 无 |

对 `DualTrajectoryAction.hpp`、`CMakeLists.txt`、launch、孪生脚本的修改也全部为自研代码。

### 2.2 仓库既有的许可状态（**历史遗留，本轮不改动，但必须在上位/公开发布前解决**）

| 包 | `package.xml` 声明 | 随包 LICENSE 文件 | 仓库内实际内容 | 判断 |
|---|---|---|---|---|
| `duco_support` | `<license>BSD</license>`，维护者 `Dave Coleman` | **有**，内容为 **Apache-2.0** | GCR5-910 网格/碰撞 STL、URDF | ⚠️ **声明（BSD）与随附 LICENSE（Apache-2.0）不一致**；资产来源为厂商机器人模型 |
| `duco_gcr5_910_moveit_config` | `<license>BSD</license>`，维护者 `siasun_duco_rc@163.com`（厂商） | **无** | 厂商机器人 URDF/SDF、world、MoveIt 配置 | ⚠️ **厂商资产，再分发授权未确认** |
| `duco_ros_driver` | **`TODO: License declaration`** | **无** | 含 `SIASUN Robotics (c) 2017-2018` 与 `Southwest Research Institute (c) 2012` 的 BSD 头 | ⚠️ **包级许可未声明**；含厂商驱动源码 |
| `duco_msg` | **`TODO: License declaration`** | **无** | 消息定义 | ⚠️ 包级许可未声明 |
| `plc_s7_bridge` / `plc_modbus_bridge` | `BSD-3-Clause`，维护者 `lsh953` | 无 | 自研 | ✅ 自研 |

### 2.3 结论与建议

1. **本轮不作为新增公开发布内容引入任何未确认授权的厂商/第三方材料。**
   新增内容仅为上述自研文件；厂商机器人模型、厂商驱动源码均为**早已存在于 `lsh953/main`** 的历史内容，
   本 PR 既不新增、也不修改它们。
2. 但 `duco_ros_driver` / `duco_msg` 的 `package.xml` 许可为 `TODO`，且 `duco_support` 声明与随附 LICENSE 冲突。
   **在公开发布（Release / 公开镜像 / 对外分发）之前**，必须由权利人补齐包级许可声明，
   并确认 `duco_gcr5_910_moveit_config` 中厂商资产（URDF/SDF/网格）的再分发授权。
   本轮只做标注，**不代替权利人做授权判断**。
3. 新增的 `InterlockPersistence.hpp` 位于 `duco_ros_driver` 包内，因此其对外许可状态受该包级声明影响——
   这是需要权利人先解决的既有问题，而不是本轮引入的。

---

## 三、Lint：本轮是否引入新问题（与历史问题区分）

### 3.1 方法

在**同一把工具、同一套参数**下，对
`github tag` 无关的两份源码树分别运行 linter 并逐文件比对：

- 当前树：`src/duco_ros_driver`（集成分支）
- 基线树：`git archive 7e7c34b src/duco_ros_driver` 展开到临时目录
- 方法校核：直接运行 `ament_uncrustify --linelength 100` 得到的失败文件集合，
  与 `colcon test` 产出的 `uncrustify.xunit.xml` **完全一致（25/25，无差异）**，
  证明该直接运行方式与被测流程等价。

### 3.2 结果

| Linter | 基线 `7e7c34b` | 当前集成分支 | 差异 |
|---|---|---|---|
| `uncrustify` | 失败 **24** / 26 文件 | 失败 **24** / 28 文件 | **集合完全相同**；本轮新增的 2 个文件**均通过** |
| `lint_cmake` | 11 处发现 | 11 处发现 | **同一条目集合**，仅因 CMakeLists 顶部新增 4 行而**行号整体 +4** |
| `cppcheck` | 28 项 skipped | 28 项 skipped | 均为 `Test Skipped due to cppcheck 2.13.0 performance issues`，**不是通过**，也不是发现 |
| `xmllint` | `package.xml` 通过 | `package.xml` 通过 | 无差异 |

**结论：本轮改动引入的新 lint 问题数为 0。**

> 过程说明：本轮新增的 `test/test_interlock_persistence.cpp` 起初有一处
> `namespace {` 换行风格差异（唯一一处），已在 `1ffdbca` 中按仓库最新测试文件的风格修正，
> 因此最终失败集合与基线**逐文件一致**。即"本轮 lint 债务增量为零"。

### 3.3 历史遗留 lint 问题（**非本轮引入，未修**）

- `uncrustify`：24 个文件不通过，涉及整包的引用/指针绑定风格、`RCLCPP_*` 宏与代码同行等，
  是仓库长期存在的全量风格偏离。**刻意不做全仓库重排**——那会产生无法审阅的巨型 diff，
  并可能与厂商源码的既有风格冲突。
- `lint_cmake`：`CMakeLists.txt` 中 `install(...)` 的空格风格与一处超长行，历史遗留。

---

## 四、编译与自动化测试报告（均在集成分支上重跑）

### 4.1 编译

```
$ source /opt/ros/jazzy/setup.bash
$ cd /home/me/duco_ros2_ws
$ colcon build --packages-select duco_ros_driver
Starting >>> duco_ros_driver
Finished <<< duco_ros_driver [18.2s]
Summary: 1 package finished [18.3s]
```

结果：**成功**（0 error / 0 warning 阻断）。工作区其余 5 个包此前已构建通过，本轮未改动其源码。

### 4.2 C++ 单元测试与集成测试（直接运行）

| 测试二进制 | 用例数 | 结果 |
|---|---|---|
| `test_interlock_persistence` | 15 | **15 / 15 PASSED** |
| `test_real_motion_interlock` | 14 | **14 / 14 PASSED** |
| `test_real_feedback_gate` | 39 | **39 / 39 PASSED** |
| `test_trajectory_validation` | 26 | **26 / 26 PASSED** |
| `test_dual_trajectory_action` | 36 | **36 / 36 PASSED** |
| **合计** | **130** | **130 / 130 PASSED** |

### 4.3 数字孪生端到端验证

```
$ bash scripts/dual_twin_verification/verify_all.sh
EXIT=0
== Phase A: Gazebo + MoveIt digital twin ==
[A] ---- 5/5 checks passed ----
[B] ---- 22/22 checks passed ----
[C] ---- 8/8 checks passed ----
== summary ==
Phase A: PASS   Phase B: PASS   Phase C: PASS
```

要点（Phase B）：S1 健康目标 `SUCCEEDED` 且落盘记录被清除；S2 取消由真实
`joint_trajectory_controller` 应答 → `status=5`（CANCELED）并锁存；
S3 锁存期间新目标被拒、人工恢复后解除；S4/S5 不健康状态 → `ABORTED`(6)、
恢复被拒直到状态恢复健康。

### 4.4 `colcon test` 汇总与归因

```
$ colcon test --packages-select duco_ros_driver
$ colcon test-result
Summary: 207 tests, 0 errors, 37 failures, 28 skipped
```

明细与归因：

| 来源 | 用例数 | 失败 | 说明 |
|---|---|---|---|
| `uncrustify.xunit.xml` | 28 | **24** | 历史遗留风格（§3.3） |
| `lint_cmake.xunit.xml` | 11 | **11** | 历史遗留（同上） |
| `cppcheck.xunit.xml` | 28 | 0 | **28 项 skipped**（`cppcheck 2.13.0 performance issues`），非通过 |
| `xmllint.xunit.xml` | 1 | 0 | `package.xml` 通过 |
| 5 个 gtest 二进制 | 130 | **0** | 功能测试全部通过 |
| CTest 汇总 `Testing/…/Test.xml` | 9 | **2** | 与上面 `lint_cmake`/`uncrustify` 是**同一批**问题的测试级重复计数 |
| **合计** | **207** | **37** | 唯一根因：**24 + 11 = 35 处历史遗留 lint**；功能失败 **0** |

**与基线的对比**：基线唯一根因数量同为 24 + 11 = 35（+2 处 CTest 重复计数 = 37）。
即 **`colcon test` 的失败数与 round 3 基线完全相同，功能失败为 0**。

---

## 五、本轮在集成分支上追加的改动（相对已审阅的 `31b2eaa`）

这部分是**审阅后新增**的，与已审阅提交分离，便于单独取舍：

### 5.1 `Result.error_string` 携带失败原因（`1ffdbca`）

- 触发原因：对 MoveIt 2.12.3 的源码核对发现，
  `follow_joint_trajectory_controller_handle.cpp` 会把
  `Result.error_string` 打印进 `Controller '...' failed with error <code>: <error_string>` 告警行；
  而本节点此前**始终留空**，导致该行丢失原因、无法区分"取消超时／执行超时／反馈未确认"。
- 改动：`finishGoal()` 在终止状态非 `SUCCEEDED` 时填充 `error_string`
  （等于该目标的中止原因，缺失时回落到锁存原因）；`SUCCESSFUL` 时保持为空，
  避免只读该字段的调用方把成功误判为失败。
- 接口影响：**不改变** Action 类型、字段集合或终止状态；只填充一个既有字段。
- 测试：`CancelTimeoutIsReportedAsAbortedNotCanceled` 增加 3 条断言
  （非空、含 `timed out`、含 `may still be executing`）。
- **若审阅后不认可该改动，可单独丢弃这一个提交，不影响其余修复。**

### 5.2 Lint 中性化

`test/test_interlock_persistence.cpp` 的 `namespace {` 换行风格对齐仓库最新测试文件，
使本轮 lint 增量为零（§3.2）。

---

## 六、Pull Request 信息

### 6.1 链接

```
https://github.com/lsh953/duco_gcr5_910/compare/main...integration/round3-safety-fixes
```

一键创建 PR：

```
https://github.com/lsh953/duco_gcr5_910/pull/new/integration/round3-safety-fixes
```

> **未能自动创建 PR**：本机没有 `gh` CLI、没有 `GITHUB_TOKEN`、没有配置 git credential helper
> （仅存在一个与本任务无关的 `ANTHROPIC_AUTH_TOKEN`），且 remote 使用 SSH 协议。
> 因此 PR 只能由人工在上述链接点击创建；分支本身已按 §1.4 推送。
> **不代用户做需要凭据的操作。**

### 6.2 PR 元信息

- **Base**：`lsh953/duco_gcr5_910` `main`
- **Compare**：`integration/round3-safety-fixes`
- **Title**：`Round 3: honest cancel timeout, durable interlock, atomic task finalization`
- **可合并方式**：本分支线性建立在 `main`（`7e7c34b`）之上，可不产生合并提交地快进合并

**PR 正文（可直接粘贴）**：

```markdown
## 变更内容

第三轮双轨迹执行安全加固，建立在已审阅分支 `codex/dual-trajectory-safety-fixes`（`31b2eaa`）之上。

### 已审阅并冻结的 4 个提交
1. `f5b2c42` 取消超时不再报告 CANCELED；联锁落盘改为原子替换；任务收尾为单一事务
2. `2608934` 第三轮修复开发报告
3. `9a6ac8d` 修正测试报告中两处措辞与引用日志
4. `31b2eaa` 在测试报告中按 linter 拆分 colcon 结果

### 本 PR 追加
5. `1ffdbca` 填充 `Result.error_string`（MoveIt 会读取该字段），并按仓库风格消除本轮唯一一处 lint 差异

## 行为变更（需上游确认）
- 取消超时由 `CANCELED`(5) 改为 `ABORTED`(6)，`error_code` 仍为 `GOAL_TOLERANCE_VIOLATED`(-5)
- 该路径经 MoveIt 2.12.3 上报的错误码由 `PREEMPTED`(-7) 变为 `CONTROL_FAILED`(-4)
- 新增参数 `real_persistence_required`（默认 `true`）、`fault_latch_file` 默认由 `/tmp/...` 改为 `""`
- 详细说明见 `docs/DUAL_TRAJECTORY_UPSTREAM_COMPATIBILITY_AND_UPGRADE.md`

## 测试
- `colcon build`：成功
- gtest：130 / 130 通过（持久化 15、联锁 14、反馈门 39、轨迹校验 26、集成 36）
- `colcon test`：207 用例 / 0 功能失败 / 37 处历史遗留 lint（与基线一致，本轮增量为 0）
- 数字孪生端到端 `verify_all.sh`：exit 0，Phase A PASS、B 22/22、C 8/8

## 未包含
- 未合并 `main`，未改写历史，未执行任何实机运动测试
- 实机验证方案见 `docs/DUCO_GCR5_910_REAL_ROBOT_VALIDATION_PLAN.md`（**未授权执行**）
```

---

## 七、交付物清单

| 交付物 | 位置 |
|---|---|
| 集成分支 | `lsh953/duco_gcr5_910` : `integration/round3-safety-fixes` |
| PR 链接（待人工创建） | 见 §6.1 |
| 代码差异报告 | 本文 §1.3；完整 diff 见 §八 |
| 编译与自动化测试报告 | 本文 §四 |
| 上游接口兼容性说明 | `docs/DUAL_TRAJECTORY_UPSTREAM_COMPATIBILITY_AND_UPGRADE.md` |
| 软件参数及行为变更说明（升级说明） | 同上 §5 |
| 实机验证方案 | `docs/DUCO_GCR5_910_REAL_ROBOT_VALIDATION_PLAN.md` |
| 本轮开发/测试报告（已审阅） | `docs/DUAL_TRAJECTORY_SAFETY_FIXES_ROUND3_REPORT.md`、`…_TEST_REPORT.md` |

---

## 八、完整 Git Diff

```bash
# 本次交付（相对个人仓库 main 的完整差异）
git diff 7e7c34b..integration/round3-safety-fixes

# 仅本轮新增（相对已审阅分支）
git diff 31b2eaa..integration/round3-safety-fixes
```

---

## 九、剩余限制与未决问题

1. **上位机 Action Client 不在本仓库**：接口文档 §2.5 的四项确认（是否读终止状态 /
   `error_code` / `error_string`、是否有自动重试）**未经核对**，必须由上位机负责人完成。
2. **`send_joint_waypoints.py` 的退出码缺口**：失败时仍返回 0，本轮**未修**（避免扩大改动面），
   已列为待办。
3. **无厂商停止确认接口**：取消超时后无法由软件证明机械臂已停止，
   这是"不确定性"的根源，只能靠物理急停与流程约束，不能靠软件弥补。
4. **`GOAL_TOLERANCE_VIOLATED` 语义重载**：刻意保留，已在兼容性文档 §3.1 显式定义，
   并明确要求上游不得据此推断"机械臂已停止"。
5. **落盘并发竞态不可确定性复现**：旧的双临界区竞态无法用测试确定性触发
   （临时回退修复后压力测试仍通过）；原子性由"单一临界区"结构保证，
   唯一可确定性区分的判据是 `ARuntimeWriteFailureBlocksRealMotion`。
6. **并发/原子性的一致性问题（round 3 原报告）未变**：仍以结构性论证为主。
7. **Lint**：24 + 11 处历史遗留，刻意不修（会产生不可审阅的巨型 diff）。本轮增量为 0。
8. **许可证**：`duco_ros_driver` / `duco_msg` 包级许可为 `TODO`，`duco_support` 声明与随附
   LICENSE 冲突，`duco_gcr5_910_moveit_config` 含厂商资产。**公开发布前必须由权利人解决**（§2）。
9. **环境限制**：`cppcheck` 全部 skipped（工具性能问题），因此"cppcheck 无发现"**不能**当作质量结论。
10. **实机验证未执行、未授权**。
