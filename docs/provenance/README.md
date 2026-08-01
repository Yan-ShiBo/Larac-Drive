# 来源、制品与永久只读归档

## 两次重构记录

- [`CLEAN_REBUILD_2026-07-28.md`](CLEAN_REBUILD_2026-07-28.md) 记录前一阶段删除旧
  代码、旧结果、继承权重和旧 Git/LFS，并建立清洁基线的历史过程。
- [`MATH_FIRST_REBUILD_2026-07-28.md`](MATH_FIRST_REBUILD_2026-07-28.md) 记录当前
  math-first 五层重构、路径迁移和验证边界。

前一文件中的目录名是当时快照，不应作为当前架构说明。当前职责以根
[`README.md`](../../README.md) 和
[`architecture/OVERVIEW.md`](../architecture/OVERVIEW.md) 为准。

## 原始归档

`接手的原始文件/` 是接手现场的永久只读证据：

- 只保留最初三份压缩包；
- 不在其中解压、写日志、缓存、校验文件或索引；
- 不作为数据、模型、Python 或 ROS 搜索路径；
- 恢复历史内容时只能解压到目录之外。

活动源码和配置不得静默读取归档内容。归档哈希最终复核由主代理在重构完成后
执行并报告。

## CertificateArtifact

数学证书的字段和状态见
[`artifacts/certificates/schema.yaml`](../../artifacts/certificates/schema.yaml)。
仓库不提交运行生成的证书系数。2026-07-30 的远程服务器结果中已有一个
独立验证的 G7 W14 q-SBC，状态为 `formal_valid_not_ready`；其文件、内容和
独立验证哈希记录在
[`EXACT_LIVE_CERTIFICATE_RUN_2026-07-30.md`](../experiments/EXACT_LIVE_CERTIFICATE_RUN_2026-07-30.md)。
不得把失败的 G5/G6/G8 候选或 sampled 诊断伪装成证书。

证书必须绑定 system、controller、scenario 和 noise 的内容哈希，记录状态
顺序/单位、表示、solver、独立验证、假设和声明资格。

## Learned models

[`artifacts/models/manifest.yaml`](../../artifacts/models/manifest.yaml) 只声明五个
`missing` 槽位。权重必须放在项目内 `artifacts/payloads/`，由
`CBF_ROVER_ARTIFACT_ROOT` 注入，并登记
SHA-256、字节数、来源和加载兼容性。

模型 `verified` 不证明控制安全，也不会把 learned score 变成 formal
certificate。

## 数据与运行结果

训练数据需要独立 dataset manifest，记录来源、处理步骤、字段、单位、划分规则和
哈希。实验结果写入项目内 `runs/`（`CBF_ROVER_RUN_ROOT`），使用不可覆盖 run ID。

图表和聚合表必须能追溯到 trace、episode summary、完整配置、Git revision、
环境和 artifact 快照。无法追溯的数字不能作为正式实验结果。

## 当前验证状态

本轮只执行目录、代码、配置、制品 schema 和文档重构。本地没有运行测试、模型、
训练、ROS2 构建或 Gazebo。最终文本级静态检查和永久归档复核已经完成，明细见
[`MATH_FIRST_REBUILD_2026-07-28.md`](MATH_FIRST_REBUILD_2026-07-28.md)。
