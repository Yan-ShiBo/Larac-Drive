# 配置真源

每个配置目录只回答一个问题。实验配置通过路径组合这些真源，不复制系统、控制器、
证书、场景、运行环境或基线参数。

| 目录 | 权威内容 |
|---|---|
| `systems/` | 动力学、状态、动作、扰动和采样周期 |
| `controllers/` | nominal controller 及其输入输出契约 |
| `certificates/` | 证书 evaluator 与 artifact 选择，不存伪造系数 |
| `scenarios/` | frame、domain、集合、障碍、episode 和 topics |
| `baselines/learned_rgbd/` | 可选 learned RGB-D 经验基线 |
| `runtimes/` | 服务器、ROS2、设备和外部存储 |
| `experiments/` | 上述配置的组合、对照与记录字段 |

当前尚未确认首个 q-SBC 研究契约，因此 `systems/`、`controllers/` 和
`certificates/` 只有目录说明，没有活动数学配置。不得用占位参数生成
“已验证证书”。
