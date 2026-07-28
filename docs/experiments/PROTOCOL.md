# 实验协议与记录

## 权威实验配置

`configs/experiments/learned_rgbd_static_obstacles.yaml` 是 learned RGB-D 静态
障碍对照实验的组合入口。它引用方法、策略、场景、运行环境和制品清单，不复制
这些领域的参数。

当前状态为 `blocked_missing_artifacts`，必须保持禁用，直到五个模型槽位均为
`verified`。

当前 YAML、通用 runner 和 JSONL recorder 是执行骨架。Gazebo reset、RGB-D
同步、完整 YAML 解析、Parquet trace、配置/环境/制品快照尚未由一个 orchestrator
串联；服务器实现并保存通过证据前，不得把本协议描述为已可运行的端到端流水线。

## 成对对照

每个场景实例必须运行两组：

- `reference_policy`：不使用安全过滤；
- `learned_rgbd_filter`：使用同一个 reference policy 加 learned RGB-D 过滤。

两组必须共享 world 布局、初始姿态、随机种子、episode 上限和终止条件。场景
配置定义 5 个 seed，每个 seed 20 个 episode。改变这组设计时只修改
`configs/scenarios/static_obstacles.yaml`，并创建新的实验 ID。

## 逐步记录

每一步至少记录：

- episode ID、seed、step、时间戳和 observation ID；
- reference action、final action 和干预范数；
- CBF-like/CLF-like 分数与违规标记；
- 最近障碍距离、reward、cost 和碰撞标记。

episode 汇总至少记录终止原因、成功、碰撞、最小障碍距离、reward/cost、干预
统计、证书违规次数和过滤不可行次数。逐步 trace 与汇总字段的唯一真源位于实验
配置的 `recording` 节。

## 可复现记录

每个运行目录必须包含：

- 完整解析后的配置快照；
- Git revision 和工作区脏状态；
- OS、ROS、Gazebo、Python、CUDA、GPU 和框架版本；
- 制品清单快照及每个文件的 SHA-256；
- 数据集/场景清单；
- 命令、标准输出、标准错误；
- 逐步 trace、episode 汇总和聚合报告。

运行输出写入 `CBF_ROVER_RUN_ROOT`，不得提交到仓库。已有运行目录不得覆盖，也
不得隐式续跑；重试必须产生新的 run ID。

## 结果声明

报告必须同时展示安全指标、任务完成度和干预代价。仅碰撞下降而成功率显著下降、
车辆停滞或干预过强，不应描述为整体改进。

所有 learned RGB-D 结论限定为特定仿真分布内的经验观察。本协议不会把统计结果
升级为形式化安全证明。
