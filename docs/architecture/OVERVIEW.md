# CBF-Rover 架构

## 目标与当前状态

CBF-Rover 是仿真优先的障碍证书研究工程。研究核心与 ROS2/Gazebo 适配严格
分层，方便后续频繁替换障碍证书方法，而不把算法绑定到某个仿真节点。

本次重构删除了继承模型权重和旧实验结果。仓库当前没有可直接执行的学习模型；
必须在服务器重新训练或从可信外部制品库注入并验证。学习型 RGB-D 方法仅作为
经验基线保留其源码和配置。

## 目录与职责

```text
CBF-Rover/
├── src/cbf_rover/
│   ├── core/                  # 稳定协议、数据结构和通用数学约定
│   ├── methods/
│   │   ├── learned_rgbd/      # 经验型 RGB-D 基线
│   │   └── <new_method>/      # 后续方法按纵向包扩展
│   ├── policies/              # 参考策略接口
│   ├── experiments/           # episode、指标和记录协议
│   ├── datasets/              # 数据模式与数据集清单
│   └── cli/                   # 薄命令行入口
├── simulation/ros2_ws/src/
│   ├── cbf_rover_description/ # 机器人描述
│   ├── cbf_rover_gazebo/      # world、模型和 Gazebo 启动
│   └── cbf_rover_ros/         # ROS2 薄适配
├── configs/
│   ├── methods/               # 方法及其数学/数值约定
│   ├── policies/              # 参考策略约定
│   ├── scenarios/             # 场景、episode 和 seed
│   ├── experiments/           # 组合关系和记录字段
│   └── runtime/               # 服务器、ROS 和外部存储
├── artifacts/manifests/       # 制品身份与来源；不放权重
├── data/                      # 外部/忽略的数据挂载点
├── runs/                      # 外部/忽略的运行产物
├── docs/
├── tests/
└── 接手的原始文件/             # 永久只读的原始压缩包
```

## 依赖方向

```text
configs ───────────────┐
                      v
CLI / ROS2 adapter -> experiments -> methods / policies -> core
         |
         +-> Gazebo interfaces

artifact manifest -> external artifact root
run recording      -> external run root
```

- `core` 不导入 Torch、ROS2、Gazebo 或硬件驱动。
- `methods` 可以依赖 `core`；学习方法可以依赖 Torch。
- `experiments` 只通过稳定接口组合方法、策略和场景。
- `cbf_rover_ros` 当前只负责外部配置/制品门禁、生命周期和命令转发，不定义
  证书数学，也不加载模型。
- Gazebo 包不反向依赖具体学习模型。
- 实车、串口和物理相机不属于当前工程范围。

## 配置真源

配置采用“一个领域一个真源”的规则，禁止在 launch、Python 默认参数和多份 YAML
中重复同一数值：

| 领域 | 权威文件 |
|---|---|
| learned RGB-D 参数与声明等级 | `configs/methods/learned_rgbd.yaml` |
| 参考策略约定 | `configs/policies/depth_ppo.yaml` |
| 场景、episode 数量和随机种子 | `configs/scenarios/static_obstacles.yaml` |
| 服务器、ROS2 与外部存储 | `configs/runtime/server_ros2.yaml` |
| 组合、对照和记录字段 | `configs/experiments/learned_rgbd_static_obstacles.yaml` |
| 模型身份、哈希和来源 | `artifacts/manifests/models.yaml` |

实验配置只引用其他领域文件，不复制其参数。命令行覆盖必须写入最终配置快照，
否则该次运行不构成可复现实验。

## 运行门禁

当前 `artifacts/manifests/models.yaml` 的五个槽位均为 `missing`，因此 learned
RGB-D 实验必须保持 `enabled: false`。只有制品达到 `verified`、外部根目录已
设置、服务器完成加载检查后才能解除门禁。

本次只完成目录、配置和文档整理，没有在本机运行 Python、模型、ROS2、
Colcon、Gazebo、训练或实验。文件存在和静态结构不能视为运行验证。

`configs/experiments/*.yaml`、`experiments/runner.py` 与 recording 契约目前是
服务器执行骨架；尚无把 YAML、Gazebo reset、传感器同步、Parquet trace 和全部
快照串成一条命令的 orchestrator。该边界必须在服务器实现并验证后才能宣称
端到端实验可运行。

## 原始归档边界

`接手的原始文件/` 只保存接手时的原始压缩包，永久只读。不得在其中解压、生成
日志、写校验缓存或放置任何新文件。需要恢复历史内容时，只能向仓库外或明确的
临时目录解压。
