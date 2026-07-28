# Tests

测试按最终架构和运行边界分层，全部留待服务器执行：

```text
tests/
├── unit/
│   ├── domain/       # 数学对象、证书与动作修改的纯领域契约
│   ├── application/  # episode runner、fail-closed 行为与指标
│   └── adapters/     # 数据分组切分等边界适配器
├── provenance/       # 原始归档、模型工件和源码路径完整性
├── math/             # 证书数学与求解器验证；仅服务器
├── integration/      # 外部工件、数据与训练/实验流水线；仅服务器
└── ros2/             # ROS2、Gazebo 与运行时回归；仅服务器
```

`unit/` 只通过 `cbf_rover.domain`、`cbf_rover.application`
和 `cbf_rover.adapters` 的公开接口验证行为。`provenance/`
使用标准库解析模型 manifest；原始归档测试带有
`requires_original_archives` 标记并校验三个固定文件及其 SHA-256。

本次只重建测试代码，没有在本机运行 Python、pytest、模型、训练、ROS2 或 Gazebo。服务器执行前，所有测试状态均为“未运行”，不得写成“通过”。

服务器应分阶段执行并保存环境、命令和完整日志：

1. unit 与 provenance；
2. math；
3. 配置外部模型工件后的 integration；
4. ROS2/Gazebo 构建及 ros2 回归。

测试通过只能支持对应环境、假设和场景下的实现验证；经验指标不构成形式化安全证明。
