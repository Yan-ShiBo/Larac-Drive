# 文档索引

## 架构

- [五层架构、依赖方向与 ROS2 边界](architecture/OVERVIEW.md)
- [ADR-0001：采用 math-first 五层架构](adr/0001-math-first-five-layer-architecture.md)

## 数学与声明

- [首个 q-SBC 研究契约模板](mathematics/RESEARCH_CONTRACT.md)
- [证据等级、运行评估与声明边界](mathematics/CLAIM_LEVELS.md)
- [证书制品 schema](../artifacts/certificates/schema.yaml)

## 基线与实验

- [learned RGB-D 经验基线](baselines/LEARNED_RGBD.md)
- [实验协议与 ScenarioSpec](experiments/PROTOCOL.md)
- [服务器验证门禁](experiments/SERVER_VALIDATION.md)

## 来源与制品

- [来源、制品与永久只读归档](provenance/README.md)
- [2026-07-28 math-first 整体重构](provenance/MATH_FIRST_REBUILD_2026-07-28.md)
- [前一阶段清洁重构历史记录](provenance/CLEAN_REBUILD_2026-07-28.md)
- [模型制品清单](../artifacts/models/manifest.yaml)
- [模型制品模式](../artifacts/models/schema.yaml)

当前没有 q-SBC 证书制品，learned 模型槽位也全部为 `missing`。本轮未在本机
运行测试、模型、ROS2、Colcon 或 Gazebo；所有运行验收留到服务器。
