# 实验协议与 ScenarioSpec

## 配置组合

实验配置只组合权威文件：

```text
system + controller + certificate + scenario + runtime
                                      │
                                      └──> experiment
```

baseline 实验可以用 `baseline method + baseline controller` 代替主线
controller/certificate，但必须复用相同 ScenarioSpec、初始状态、seed、终止条件
和记录字段。

当前：

- `configs/systems/`、`controllers/`、`certificates/` 尚无活动数学配置；
- `configs/experiments/learned_rgbd_static_obstacles.yaml` 是默认禁用的经验对照
  骨架；
- 首个 q-SBC 实验必须等待研究契约确认。

## ScenarioSpec

[`configs/scenarios/static_obstacles.yaml`](../../configs/scenarios/static_obstacles.yaml)
是静态障碍场景的唯一逻辑真源，包含 frame/units、domain、robot footprint、
initial/goal/unsafe sets、三障碍几何、episode 和 topics。

所有消费者必须使用同一份解析快照：

```text
ScenarioSpec
├── exact state reset and set membership
├── Gazebo obstacle-spawner projection
├── RViz markers
├── goal / distance / collision metrics
└── run configuration snapshot
```

Gazebo `base.world` 只提供地面和太阳。障碍不得重新硬编码在 world、launch 或
spawner 默认参数中。

动态公路实验使用另一份独立真源：
[`dynamic_road_random_obstacles.yaml`](../../configs/scenarios/dynamic_road_random_obstacles.yaml)。
它不替换静态 P1/D1，也不继承静态 q-SBC。随机障碍的完整隐藏计划由 seed
一次性生成并绑定哈希；障碍进入声明的感知距离后才成为在线 CBF 约束和可视化
对象。

## 运行模式

目标应用支持三类职责明确的模式：

| 模式 | 动作 | 证书 | 用途 |
|---|---|---|---|
| unmodified baseline | 直接执行 nominal action | 可无 evaluator | 任务基线 |
| monitor-only | 不修改 nominal action | 每步评估并记录 | 隔离证书行为 |
| filtered | modifier 提出动作，guard 审查 | 每步评估并记录 | 闭环干预 |
| replay | 不执行环境动作 | 重放相同 evaluator | 离线审计 |

`application/episode_runner.py` 仍只负责 unmodified 和带 modifier 的有界 episode
骨架。M1 exact 的 monitor-only evaluator、逐步 trace、certificate replay 和
ScenarioSpec resolver 已由专用入口实现；ROS2 composition 与 RViz marker 代码也
已接入，但尚未在具备 ROS2 Humble 的服务器验证。通用 ExecutionGuard 仍需后续
实现，文档不会把未验证的 ROS2 路径描述为已通过。

modifier 报告 `feasible: false` 时必须在调用 environment 前停止。未来 fallback
只能来自明确 guard policy；零动作不是默认安全策略。

动态公路入口属于 `filtered`：nominal 双车道控制器先给出圆盘内动作，在线距离
CBF 再求半空间与 S1 速度圆盘交集上的最小改变量。求解不可行时立即以
`cbf_action_set_infeasible` 结束，不进行 clipping、零动作 fallback 或静默重试。

动态公路的多 seed 实验必须把开发发现集与 held-out 验证集分开。批量统计只保存
成功 episode 的确定性摘要与 trace 哈希，失败 episode 额外保存完整 trace。验证
开始后若修改 controller、CBF、场景或执行语义，该验证结果立即失效，必须换用新
的 held-out seed 目录。

## Exact 与 Gazebo

每个数学证书实验先通过 exact backend：

- 状态按研究契约直接推进；
- 相同 seed 产生相同噪声与轨迹；
- `W`、期望、残差和集合归属可与手算结果比较；
- state/action/config/artifact 不兼容时 fail closed。

随后才投影到 Gazebo：

- 场景几何来自同一 ScenarioSpec；
- odometry encoder 与 command tracker 必须版本化；
- 记录 requested、modified、commanded 和 executed action；
- tracking error 未证明时，结果只能是 `empirical` 或 `diagnostic`。

exact 或 Gazebo 成功运行都不自动产生 formal claim。

## 逐步记录

每一步至少记录：

- run、episode、step、seed、时间戳和后端；
- ScenarioSpec、system、controller、noise 和 certificate artifact identity/hash；
- observation/state schema、frame 和 applicability；
- nominal、modified、commanded、executed action；
- certificate condition、value、violation、适用状态和数值状态；
- modifier feasibility、intervention norm 和 guard decision；
- goal、unsafe-set membership、最近障碍距离、碰撞、reward 和 cost；
- termination 或错误原因。

特定 q-SBC 字段在研究契约确认后加入专用 telemetry，可能包括 `W(x)`、一步期望、
残差、`q` 和 `epsilon`。未确认公式前不得伪造这些字段。

## Episode 汇总

至少汇总：

- 成功、碰撞、unsafe entry、timeout 和 runtime error；
- 最小障碍距离和 goal time；
- certificate satisfied/violated/not-applicable/numerical-error 计数；
- modifier intervention/feasibility 和 guard rejection；
- reward/cost 与控制代价；
- exact/Gazebo backend 与 claim level。

仅碰撞下降但车辆停滞、成功率下降或 guard 大量拒绝，不能描述为整体提升。

## 可复现性

每个不可覆盖的 run 目录必须保存：

- 完整解析配置和原始配置哈希；
- Git revision、branch/tag 和工作区脏状态；
- OS、Python、ROS、Gazebo、CUDA、GPU、solver 和框架版本；
- certificate/model artifact 快照及所有外部文件 SHA-256；
- 命令、标准输出和标准错误；
- 逐步 trace、episode 汇总、聚合表和图。

本地输出位于项目内 `runs/`（`CBF_ROVER_RUN_ROOT`）。远程服务器结果需同步回该目录；
聚合结果必须可回溯到原始 trace。

## Learned 对照

learned RGB-D 只有在五个模型槽位 verified 且服务器加载检查通过后才能运行。它
与主方法共享同场景、同初始状态、同 seed，但始终按 empirical 报告，除非未来
另有独立数学证明。

本轮未在本机运行任何实验。
