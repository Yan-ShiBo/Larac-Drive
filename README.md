# CBF-Rover

CBF-Rover 是一个以 Gazebo 仿真为运行边界、以障碍证书和安全过滤为研究核心的
工程。项目不包含实车驱动，也不把学习型 CBF/CLF-like 分数描述为形式化安全
证明。

> `接手的原始文件/` 是接手现场三份压缩包的永久只读归档。不要在其中新增、
> 删除、重命名或修改任何内容；开发、模型、数据和运行结果都必须位于目录外。

## 当前状态

- 旧 ROS1、历史结果、继承模型、实车代码和重复 ROS2 实现已退出活动架构。
- 纯 Python 核心负责证书协议、方法实现、实验生命周期和记录契约。
- ROS2 工作区只保留机器人描述、Gazebo 场景和薄 I/O 适配。
- 仓库不携带模型权重；五个 learned RGB-D 制品槽位当前均为 `missing`。
- learned RGB-D 方法和对应实验默认禁用，直到服务器提供可信制品并完成验证。
- 本机不运行测试、模型、训练、Colcon 或 Gazebo；运行验收留到服务器。

## 目录

```text
CBF-Rover/
├── src/cbf_rover/
│   ├── core/                   # 无 ROS/Torch 的稳定研究契约
│   ├── methods/                # 按方法纵向组织的证书与过滤器
│   ├── policies/               # 参考策略协议与适配
│   ├── experiments/            # episode、记录和对照实验
│   ├── datasets/               # 数据结构与划分
│   └── cli/                    # 数据处理与训练入口
├── simulation/ros2_ws/src/
│   ├── cbf_rover_description/  # 机器人描述
│   ├── cbf_rover_gazebo/       # world、模型、场景和 launch
│   └── cbf_rover_ros/          # ROS2 消息/生命周期适配
├── configs/
│   ├── methods/
│   ├── policies/
│   ├── scenarios/
│   ├── experiments/
│   └── runtime/
├── artifacts/manifests/        # 外部模型身份和来源，不存权重
├── docs/
├── tests/                      # 仅在服务器执行
├── data/                       # 本地/服务器数据，Git 忽略
├── runs/                       # 实验输出，Git 忽略
└── 接手的原始文件/             # 永久只读
```

## 架构边界

- `core` 不依赖 Torch、ROS、Gazebo 或文件路径。
- `methods` 可以依赖学习框架或求解器，但不能发布 ROS 消息。
- `cbf_rover_ros` 当前只提供 manifest 门禁和生命周期控制的外部命令转发；
  它不加载模型，也不把收到的命令自行声明为安全。
- 场景配置拥有 world、障碍布局、episode 条件和随机种子。
- 模型从仓库外 `CBF_ROVER_ARTIFACT_ROOT` 注入，并按 manifest 校验。
- 运行结果写入仓库外 `CBF_ROVER_RUN_ROOT`，不得覆盖已有 run。

## 阅读顺序

1. [架构与依赖方向](docs/architecture/OVERVIEW.md)
2. [障碍证书方法规范](docs/methods/README.md)
3. [learned RGB-D 经验基线](docs/methods/LEARNED_RGBD.md)
4. [实验协议](docs/experiments/PROTOCOL.md)
5. [模型与来源治理](docs/provenance/README.md)
6. [服务器验证门禁](docs/experiments/SERVER_VALIDATION.md)
7. [本轮清洁重构记录](docs/provenance/CLEAN_REBUILD_2026-07-28.md)

## 服务器阶段

后续服务器阶段才会安装包、准备或训练模型、运行测试、构建 ROS2、启动 Gazebo
和执行成对实验。任何缺失制品、哈希不一致、配置不完整或 episode reset
失败都必须 fail closed。

当前 CLI 已覆盖数据合并、证书数据集生成和 VAE/MDRNN/CBF/CLF 训练；实验 YAML
与 recording 模块仍是服务器阶段需要接入仿真适配器的执行骨架，不是已经可运行
的端到端实验编排器。

静态文件存在、模型成功加载、ROS2 成功构建或仿真冒烟均不自动升级为安全性
结论。learned 方法的结果始终限定为记录环境和仿真分布内的经验观察。
