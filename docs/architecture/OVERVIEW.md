# Larac-Drive 架构

## 车辆控制与验证的目标架构

Larac-Drive 计划采用两条相互衔接的控制路径。

- **道路闭环控制。** Frenet 无侧滑模型以纵向加速度和车辆曲率为控制输入。SAC 给出名义动作，含控制平滑代价的 QP 根据纵向安全时距的鲁棒 CBF 和横向道路边界的 HOCBF 修正动作。低速及 QP 不可行时进入紧急制动。正常跟驰、前车急刹、邻车切入三类工况用于和 IDM、MPC 对比，测量跟踪误差、约束违约率、控制变化和推理及求解耗时。
- **同车道随机制动下的纵向应急控制。** 离线用多项式近似应急策略并评估 PAC 近似误差，在三维纵向闭环上以 SOS/SDP 求解 SBC，依期望下降条件优化控制器系数。在线只在证书覆盖的状态域内使用验证后的多项式控制器。以 Monte Carlo 估计及置信区间与证书下界对照，评估其保守程度。

具体研究范围、技术路线和预期结果见[项目方向](../PROJECT_DIRECTION.md)。

## 已有代码的五层结构

以下目录和接口来自 CBF-Rover 阶段，描述已有的二维数学模型、差速车仿真、证书求解与验证工具。原有实现沿用 `cbf_rover` 包名。

```text
src/cbf_rover/
├── domain/
│   ├── state.py                  # 传输无关的 observation/state
│   ├── action.py                 # 有标签的有限维 action
│   ├── system.py                 # 动力学预测端口
│   ├── controller.py             # nominal controller 端口
│   ├── certificates/             # 只评估、不改动作
│   ├── modifiers.py              # 可选动作修改及可行性
│   └── claims.py                 # 显式证据标签
├── methods/
│   ├── controllers/              # controller 实现
│   ├── polynomial_sbc/           # 多项式控制器、证书求解与验证
│   └── baselines/learned_rgbd/   # learned 推理与经验 modifier
├── application/
│   ├── episode.py                # 生命周期
│   ├── episode_runner.py         # 有界 episode 编排
│   ├── metrics.py                # 运行指标
│   └── telemetry.py              # 存储无关记录
├── adapters/
│   ├── datasets/                 # 数据格式与处理
│   ├── storage/                  # JSONL 等外部存储
│   └── training/learned_rgbd/    # 服务器端 baseline 训练/I/O
└── entrypoints/cli/              # 参数解析和应用调用
```

`methods/polynomial_sbc/` 包含多项式策略拟合、SOS 建模和独立证书验证方法。G7 W14 已在原二维模型上通过独立验证，其适用条件与数值下界见[二维 M1 实验记录](../experiments/EXACT_LIVE_CERTIFICATE_RUN_2026-07-30.md)。

## 依赖规则

```text
                    ┌──────────────┐
                    │ entrypoints  │
                    └──┬─────┬───┬─┘
                       v     v   v
        ┌──────────────┐  ┌─────────┐  ┌─────────┐
        │ application  │<─│ adapters│─>│ methods │
        └──────┬───────┘  └────┬────┘  └────┬────┘
               └──────────┬─────┴────────────┘
                          v
                     ┌─────────┐
                     │ domain  │
                     └─────────┘
```

- `domain` 是最内层，只使用标准库和明确的数值类型。
- `methods` 只依赖 domain contract；需要 Torch 或求解器的依赖留在具体方法包。
- `application` 通过端口组合流程，不识别 ROS message 或 Gazebo service。
- `adapters` 依赖 application/domain/method 端口，把外部数据、存储、训练、
  exact backend、ROS/Gazebo 格式转换为可组合实现。
- `entrypoints` 连接 application、methods 与 adapters，不复制算法。
- 配置和制品元数据在 composition boundary 解析，解析结果必须快照化。

禁止的反向依赖包括 domain 导入 Torch/ROS、application 导入具体 adapter、
methods 导入 adapter、Gazebo 包导入 learned 网络、方法直接发布 topic，以及
CLI 直接实现证书数学。

## 证书与动作是两条链

```text
state + nominal action
        │
        ├──> certificate evaluator ──> evaluation / telemetry
        │
        └──> optional action modifier ──> candidate action
                                              │
                                              v
                                      execution guard
                                              │
                                              v
                                          environment
```

evaluator 只回答证书条件在给定状态/动作下如何，不修改动作。modifier 可以提出
新动作，但必须报告可行性和干预量。guard 决定不可行、数值错误、配置不兼容或
超时如何 fail closed。

当前 application runner 已支持无 modifier 的原始 controller 路径，以及带
modifier 的 fail-closed 路径。它还不是完整的 q-SBC monitor：在加入具体
certificate evaluator、专用 telemetry 和 replay 前，不应把无 modifier 路径
描述为证书监控已经完成。

## 离线与在线

```text
离线：
ResearchContract
  -> synthesis inputs
  -> solver
  -> CertificateArtifact
  -> independent validator

在线：
resolved configs + validated artifact
  -> state adapter
  -> controller
  -> evaluator
  -> optional modifier
  -> guard
  -> environment
  -> trace
```

在线运行不重新求解证书。它只加载兼容制品、评估条件、执行明确策略并记录事实。
任何 system/controller/scenario/noise 哈希不匹配都会取消声明资格。

## 一个场景真源

[`configs/scenarios/static_obstacles.yaml`](../../configs/scenarios/static_obstacles.yaml)
同时描述：

- frame 和单位；
- domain、初始集、目标集和 unsafe sets；
- rover footprint、安全 inflation 和初始 pose；
- 三个障碍物的 3D 几何；
- exact/Gazebo backend 绑定；
- episode、seed、终止条件和 topics。

同一份解析结果必须驱动：

```text
ScenarioSpec
├── exact environment
├── Gazebo obstacle projection
├── RViz markers
├── distance/collision/goal metrics
└── run snapshot
```

`cbf_rover_sim/worlds/base.world` 只包含基础地面和太阳，不包含研究障碍。Gazebo
launch 与 spawner 不得再提供示例障碍默认值；application-level scenario runner
负责把 `ScenarioSpec` 投影并注入。

## 两个仿真后端

| 后端 | 用途 | 允许的结论 |
|---|---|---|
| exact | 按研究契约直接推进数学状态，检查 evaluator 与轨迹 | 实现一致性、数值和经验结果 |
| Gazebo | 传感器、差速运动、插件、时间和 ROS 集成 | 记录环境中的经验或诊断结果 |

exact simulator 不是证明器。形式声明仍需要完整定理、求解证据、假设和独立验证。
若证书针对二维单积分器，而 Gazebo 执行差速车命令，必须显式建模状态编码、低层
跟踪器和误差界；没有误差界时不能把 Gazebo 结果标成 formal。

## ROS2 三包

```text
simulation/ros2_ws/src/
├── cbf_rover_description/  # Xacro 几何与传感器 frame，无 Gazebo 插件
├── cbf_rover_sim/          # base.world、仿真插件、模型和障碍投影入口
└── cbf_rover_runtime/      # ROS2 状态/命令运行适配边界
```

`cbf_rover_runtime` 当前只有默认禁用的 external command relay。它只把外部
`TwistStamped` 转成 `Twist`，不验证命令、模型或证书，不修改动作，也不建立
安全声明。真正的 runtime evaluator 必须等 Python application contract 完成后
再接入。

## 配置与制品

配置目录按 system、controller、certificate、scenario、baseline、runtime 和
experiment 分域。证书制品与 learned 模型制品分开：

- [`artifacts/certificates/schema.yaml`](../../artifacts/certificates/schema.yaml)
  绑定数学兼容性、系数、求解、验证、假设和声明资格；
- [`artifacts/models/manifest.yaml`](../../artifacts/models/manifest.yaml)
  绑定外部模型身份、来源和加载兼容性。

`verified` 模型不等于安全。合法 certificate artifact 也只有在运行配置完全
匹配且假设成立时才有对应声明资格。

## CBF-Rover 阶段的验证记录

本轮没有在本机运行 pytest、包导入、模型加载、训练、ROS2、Colcon 或 Gazebo。
目录和静态契约存在不代表运行通过。服务器验收见
[`SERVER_VALIDATION.md`](../experiments/SERVER_VALIDATION.md)。

永久只读原始归档不参与源码、模型或运行时路径。恢复历史内容时只能解压到归档
目录之外。
