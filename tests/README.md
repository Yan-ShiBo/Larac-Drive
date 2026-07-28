# Tests

测试按运行边界分层，全部留待服务器执行：

```text
tests/
├── unit/          # 纯 Python 公共契约、分组切分、指标与 episode 生命周期
├── provenance/    # 原始归档、模型工件和源码边界的静态完整性
├── integration/   # 外部工件、数据与训练/实验流水线；仅服务器
└── ros2/          # ROS2、Gazebo 与桥接回归；仅服务器
```

`unit/` 只通过公开接口验证行为，不依赖 ROS、Gazebo 或真实模型。`provenance/` 使用 Python 标准库；其中原始归档测试会校验目录只含三个固定文件及其 SHA-256，模型清单测试不依赖 PyYAML。

本次仅完成测试代码重建，未在本机运行 Python、pytest、模型、训练、ROS2 或 Gazebo。服务器执行前，所有测试状态均为“未运行”，不能写成“通过”。建议服务器按以下阶段执行并保存完整环境、命令和日志：

1. `python -m pytest tests/unit tests/provenance`
2. `python -m pytest tests/integration`（待新增集成用例并配置外部工件后）
3. ROS2/Gazebo 构建与 `tests/ros2` 回归（按服务器验证文档）

测试通过只能支持对应环境与场景下的实现验证；学习型分数和仿真指标仍属于经验性证据，不构成形式化安全证明。
