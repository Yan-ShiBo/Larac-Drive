# 文档索引

以下文档描述重构后的当前工程：

- [架构与依赖边界](architecture/OVERVIEW.md)
- [障碍证书方法](methods/README.md)
- [learned RGB-D 经验基线](methods/LEARNED_RGBD.md)
- [实验协议与记录](experiments/PROTOCOL.md)
- [服务器验证门禁](experiments/SERVER_VALIDATION.md)
- [来源、制品与原始归档](provenance/README.md)
- [2026-07-28 清洁重构记录](provenance/CLEAN_REBUILD_2026-07-28.md)
- [模型制品清单](../artifacts/manifests/models.yaml)
- [模型制品清单规范](../artifacts/manifests/schema.yaml)

当前没有活动模型权重，learned RGB-D 实验处于
`blocked_missing_artifacts`。所有运行、训练和仿真验证均留待服务器执行；本地
未进行运行验证。

旧的 handover、首次导入盘点、旧 ROS2 接口和旧测试计划不再是当前工程说明，
应在本次清理阶段移除。历史恢复来源仅为永久只读的 `接手的原始文件/`。
