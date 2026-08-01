# Server-only integration tests

本目录预留给 domain、application、adapters 与外部模型工件之间的集成验证。所有测试只在服务器环境运行，本机不执行 Python、模型或训练任务。

计划覆盖：

- 通过项目内 `artifacts/payloads/`（`CBF_ROVER_ARTIFACT_ROOT`）注入并校验五个模型工件；
- 缺失、哈希不匹配或接口不兼容的工件必须 fail closed；
- 数据集适配器、训练入口、检查点加载、episode runner 和遥测记录的端到端契约；
- 固定场景、固定随机种子与完整日志下的可复现实验。

当前目录没有可宣称通过的集成结果。只有服务器实际执行并保存环境、命令、日志和产物后才能记录为通过；集成测试结果仍不构成形式化安全证明。
