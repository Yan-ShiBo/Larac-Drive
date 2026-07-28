# CBF-Rover

CBF-Rover 是一个以障碍证书为研究对象、以仿真为验证环境的 math-first 工程。
主线是“数学系统与证书 → 应用编排 → exact 仿真 → ROS2/Gazebo 投影”；learned
RGB-D 只保留为默认禁用的经验基线。项目没有实车，也不包含硬件驱动主线。

> [`接手的原始文件/`](接手的原始文件/) 永久只读，只保存接手时的三份压缩包。
> 不得在其中解压、新增、删除、重命名或修改任何内容。所有开发、制品、数据和
> 运行结果都必须位于该目录之外。

## 当前事实

- 已建立重构前基线：commit `1e5a8d8`、tag `pre-math-first-refactor`。
- Python 代码按 `domain → methods → application/adapters → entrypoints` 重组。
- ROS2 工作区只保留 `description / sim / runtime` 三个具体职责包。
- [`static_obstacles.yaml`](configs/scenarios/static_obstacles.yaml) 是 exact、
  Gazebo、RViz 与指标应共同消费的逻辑 `ScenarioSpec`。
- learned RGB-D 的五个模型槽位全部为 `missing`；reference controller、动作
  codec、场景 resolver 和实验 orchestrator 也未实现，方法与实验保持禁用。
- 当前没有 q-SBC 证书制品，首个公式和声明语义仍需用户确认。
- 本轮没有在本机运行测试、Python 入口、模型、ROS2、Colcon 或 Gazebo。

因此，当前交付物是清晰的研究边界、代码骨架、配置真源和制品契约，不是已经
通过运行验收的证书系统。

## 目录

```text
CBF-Rover/
├── src/cbf_rover/
│   ├── domain/                     # 纯数学/领域类型与端口
│   ├── methods/
│   │   ├── controllers/            # nominal controller 实现
│   │   ├── polynomial_sbc/         # q-SBC 研究边界；实现等待契约确认
│   │   └── baselines/learned_rgbd/ # 可选经验基线
│   ├── application/                # episode、编排、指标与 telemetry
│   ├── adapters/
│   │   ├── datasets/               # 数据格式适配
│   │   ├── storage/                # 记录存储适配
│   │   └── training/learned_rgbd/  # 服务器端 baseline 训练适配
│   └── entrypoints/cli/            # 薄 CLI
├── simulation/ros2_ws/src/
│   ├── cbf_rover_description/      # 无仿真插件的机器人描述
│   ├── cbf_rover_sim/              # Gazebo world、插件与场景投影入口
│   └── cbf_rover_runtime/          # ROS2 运行适配边界
├── configs/
│   ├── systems/                    # 动力学与噪声
│   ├── controllers/                # nominal controller
│   ├── certificates/               # evaluator 与证书选择
│   ├── scenarios/                  # 唯一逻辑场景真源
│   ├── baselines/learned_rgbd/     # learned 配置
│   ├── runtimes/                   # 服务器、ROS2 与存储
│   └── experiments/                # 组合与记录
├── artifacts/
│   ├── certificates/               # 证书 schema；当前无证书
│   └── models/                     # 模型 manifest/schema；不存权重
├── docs/
│   ├── architecture/
│   ├── mathematics/
│   ├── baselines/
│   ├── experiments/
│   ├── provenance/
│   └── adr/
├── tests/                           # 后续仅在服务器运行
└── 接手的原始文件/                 # 永久只读归档
```

`data/`、`runs/`、ROS 构建目录和外部权重不属于版本化源码，按忽略规则或外部根
目录管理。

## 五层依赖

```text
entrypoints ──> application ──> domain
     │              ▲             ▲
     ├─> adapters ──┘             │
     │       └────────────────────┤
     └─> methods ─────────────────┘

ROS2 / Gazebo ──> adapters
configs + artifacts ──> composition boundary
```

核心规则：

- `domain` 不导入 Torch、ROS2、Gazebo、文件路径或具体存储。
- `methods` 只依赖 domain，实现数学方法、控制器或明确标注的 baseline，不发布
  ROS 消息，也不导入数据/存储 adapter。
- `application` 只通过 domain 端口编排控制器、证书 evaluator、可选 action
  modifier、guard 和环境。
- `adapters` 依赖 application/domain/method 端口，负责数据、存储、训练、
  exact/Gazebo/ROS 等外部转换。
- `entrypoints` 是薄 composition root，连接 application、methods 与 adapters，
  不承载研究数学。
- certificate evaluator 与 action modifier 分开；不可行结果必须 fail closed，不能
  默认为零动作安全。

完整依赖规则见
[`docs/architecture/OVERVIEW.md`](docs/architecture/OVERVIEW.md)。

## 配置与制品真源

| 内容 | 权威位置 |
|---|---|
| 系统、控制器和证书选择 | `configs/systems/`、`controllers/`、`certificates/` |
| frame、domain、集合、三障碍、episode、topics | `configs/scenarios/static_obstacles.yaml` |
| learned RGB-D 与 depth PPO | `configs/baselines/learned_rgbd/` |
| 服务器与 ROS2 | `configs/runtimes/server_ros2.yaml` |
| learned 对照组合与记录字段 | `configs/experiments/learned_rgbd_static_obstacles.yaml` |
| 数学证书制品契约 | `artifacts/certificates/schema.yaml` |
| learned 模型身份与来源 | `artifacts/models/manifest.yaml` |

`ScenarioSpec` 必须投影到 exact simulator、Gazebo 障碍生成、RViz marker、距离/
碰撞指标和运行快照。禁止在 launch、world、节点默认值或第二份 YAML 中重新
维护同一障碍几何。

模型从仓库外 `CBF_ROVER_ARTIFACT_ROOT` 注入，运行结果写入仓库外
`CBF_ROVER_RUN_ROOT`。模型身份与加载验证不等于控制安全证明。

## 研究顺序

1. 用户确认
   [`RESEARCH_CONTRACT.md`](docs/mathematics/RESEARCH_CONTRACT.md) 中的系统、
   噪声、集合、q-SBC 公式和概率结论。
2. 建立独立于 ROS/Torch 的 system、controller、certificate evaluator 和 exact
   simulator 最小闭环。
3. 由离线求解/验证流程生成
   [`CertificateArtifact`](artifacts/certificates/schema.yaml)，运行时只加载和
   评估。
4. 让同一个 `ScenarioSpec` 驱动 exact、Gazebo、RViz 和 metrics。
5. 最后恢复 learned RGB-D 作为同场景、同 seed 的经验对照。

exact 仿真用于检查实现和实验行为，本身不会产生形式证明。差速车 Gazebo 结果
若没有经过证明的跟踪误差界，只能作为经验或诊断证据。

## 阅读顺序

1. [架构与依赖方向](docs/architecture/OVERVIEW.md)
2. [首个 q-SBC 研究契约模板](docs/mathematics/RESEARCH_CONTRACT.md)
3. [证据与声明等级](docs/mathematics/CLAIM_LEVELS.md)
4. [math-first 架构 ADR](docs/adr/0001-math-first-five-layer-architecture.md)
5. [ScenarioSpec 与实验协议](docs/experiments/PROTOCOL.md)
6. [learned RGB-D 经验基线](docs/baselines/LEARNED_RGBD.md)
7. [服务器验证门禁](docs/experiments/SERVER_VALIDATION.md)
8. [本轮迁移与来源记录](docs/provenance/MATH_FIRST_REBUILD_2026-07-28.md)

任何静态文件存在、模型成功加载、测试通过、ROS2 构建成功或仿真冒烟，都只能
支持其实际覆盖的结论。声明等级必须由数学假设、兼容哈希、求解证据和独立验证
共同决定。
