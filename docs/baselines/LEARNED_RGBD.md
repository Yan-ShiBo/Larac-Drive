# Learned RGB-D 经验基线

## 定位

learned RGB-D 是从接手工程整理出的可选经验对照。它研究 RGB-D 表征、潜在动力学
和动作修正的组合行为，不是本项目的数学证书主线。

```text
RGB-D -> VAE latent -> MDRNN next latent -> CBF-like / CLF-like scores
depth -> depth PPO nominal action --------------------------> action modifier
```

推理模型和 modifier 位于 `src/cbf_rover/methods/baselines/learned_rgbd/`；
涉及数据集、文件输出、绘图和训练运行的代码隔离在
`src/cbf_rover/adapters/training/learned_rgbd/`。权威配置位于
[`configs/baselines/learned_rgbd/`](../../configs/baselines/learned_rgbd/)。
learned 特有类型不能反向注入 domain、application 或 Gazebo。

## 当前不可执行

当前缺少五个外部权重：

- depth PPO reference policy；
- RGB-D VAE；
- RGB-D MDRNN；
- learned CBF-like 网络；
- learned CLF-like 网络。

[`artifacts/models/manifest.yaml`](../../artifacts/models/manifest.yaml) 将所有槽位
标为 `missing`，方法和实验配置保持 `enabled: false`。不得用随机权重、同名文件、
原始归档路径或未登记 checkpoint 产生基线结果。

此外，depth PPO reference controller loader、observation adapter、归一化动作到
Gazebo `Twist` 的 codec、ScenarioSpec resolver 和实验 orchestrator 尚未实现。
因此即使五个权重被注入，该对照实验也不会自动变为可运行状态；完整 blocker
列表记录在实验与 reference-controller 配置中。

恢复只能发生在服务器：

1. 通过可追溯流程重新训练；或
2. 从可信外部存储注入，并补齐 URI、SHA-256、字节数、来源和兼容信息。

原始归档中的旧权重仅用于来源研究，不会自动成为活动模型。

## 模型加载门禁

运行时必须从项目内 `artifacts/payloads/`（`CBF_ROVER_ARTIFACT_ROOT`）和版本化
manifest 构造 checkpoint set。
需要同时满足：

- `artifact_set.status: ready`；
- 所有必需槽位为 `verified`；
- `artifact://` URI 解析后仍位于 artifact root；
- 字节数和 SHA-256 完全匹配；
- framework、architecture、loader、input/output contract 匹配；
- 目标服务器完成实际加载和最小输入输出检查。

`cbf_rover_runtime` 中的 external command relay 只做 ROS 消息类型转换，不验证
manifest、模型或命令。它不是 learned action-modifier runtime。

## 证据边界

learned RGB-D 永远不能仅凭模型加载成功升级为 formal：

- CBF-like “越低越安全”只是 inherited 工程约定；
- VAE 表征误差和 MDRNN 预测误差没有全局验证界；
- 高斯混合均值可能掩盖尾部模式；
- 有限步 Adam action modifier 不保证可行；
- latent safe set 没有独立几何或可达性证明；
- 当前没有把差速 Gazebo dynamics 纳入数学保证。

因此结果只可描述为“在指定制品、配置、场景和 seed 下观察到的经验行为”。

## Episode 与记录

MDRNN 隐状态必须在每个 episode 开始时重置。环境执行动作后，modifier 必须使用
实际 `executed_action` 更新隐状态；如果环境裁剪命令，不能拿请求动作代替。

记录至少包含：

- episode/step/seed 和 observation identity；
- nominal、modified、commanded、executed action；
- CBF-like/CLF-like 分数、干预量、可行性；
- 最近障碍距离、碰撞、reward、cost 和终止原因；
- 完整配置、环境、代码 revision 和模型 manifest 快照。

同一场景对照必须共享 ScenarioSpec、初始状态和 seed。

## 验证状态

本轮只迁移源码、配置和文档边界。没有在本机加载模型、运行推理、训练、测试、
构建 ROS2 或启动 Gazebo。
