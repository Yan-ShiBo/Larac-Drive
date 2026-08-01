# CBF-Rover

## Dynamic road live CBF

The dynamic-road path is intentionally separate from the fixed-geometry
polynomial q-SBC path:

- authoritative scenario:
  [`dynamic_road_random_obstacles.yaml`](configs/scenarios/dynamic_road_random_obstacles.yaml);
- mathematical and claim contract:
  [`DYNAMIC_ROAD_ONLINE_CBF_CONTRACT.md`](docs/mathematics/DYNAMIC_ROAD_ONLINE_CBF_CONTRACT.md);
- pure geometry: `src/cbf_rover/domain/dynamic_road.py`;
- runtime filter: `src/cbf_rover/methods/online_cbf/`;
- exact seeded simulation and live plot:
  `src/cbf_rover/adapters/exact/dynamic_road.py`.

Server command:

```bash
python -m cbf_rover.entrypoints.research simulate-dynamic-road-cbf-live \
  --config configs/scenarios/dynamic_road_random_obstacles.yaml \
  --output-dir /absolute/new/result/directory \
  --seed 7302026
```

The animation reveals each random obstacle only when it enters the configured
appearance distance and updates the robust distance-CBF margin, constraint
residual, and action-filter intervention at 20 Hz. This is a runtime action
filter and empirical simulation result; it does not inherit or claim the
fixed P1/D1 q-SBC probability bound.

The next empirical gate uses the predeclared discovery and held-out catalogs
in
[`dynamic_road_multiseed.yaml`](configs/experiments/dynamic_road_multiseed.yaml):

```bash
python -m cbf_rover.entrypoints.research \
  simulate-dynamic-road-cbf-batch \
  --batch-config configs/experiments/dynamic_road_multiseed.yaml \
  --catalog discovery \
  --output-dir /absolute/new/result/directory
```

See
[`DYNAMIC_ROAD_MULTISEED_PROTOCOL.md`](docs/mathematics/DYNAMIC_ROAD_MULTISEED_PROTOCOL.md)
for seed separation, failure-trace retention, empirical interval semantics,
and the exact replay gate.

### Differential-drive lifted online CBF

The current rover-facing exact path extends the planar action filter to the
differential-drive state `[x, y, theta]` and command `[v, omega]`. A
`0.15 m` lookahead point is protected by the same road and obstacle CBFs, while
the heading-dependent reachable set, `0 <= v <= 0.5 m/s`, and
`|omega| <= 1.5 rad/s` are included inside the projection. The accepted command
therefore requires no post-CBF clipping.

- V2 single-run configuration:
  [`dynamic_road_differential_v2.yaml`](configs/experiments/dynamic_road_differential_v2.yaml);
- multi-seed configuration:
  [`dynamic_road_differential_multiseed.yaml`](configs/experiments/dynamic_road_differential_multiseed.yaml);
- mathematical contracts:
  [`DYNAMIC_ROAD_DIFFERENTIAL_CONTRACT.md`](docs/mathematics/DYNAMIC_ROAD_DIFFERENTIAL_CONTRACT.md)
  and
  [`DYNAMIC_ROAD_DIFFERENTIAL_CONTRACT_V2.md`](docs/mathematics/DYNAMIC_ROAD_DIFFERENTIAL_CONTRACT_V2.md);
- predeclared stress protocol:
  [`DYNAMIC_ROAD_DIFFERENTIAL_MULTISEED_PROTOCOL.md`](docs/mathematics/DYNAMIC_ROAD_DIFFERENTIAL_MULTISEED_PROTOCOL.md).

Server commands:

```bash
python -m cbf_rover.entrypoints.research \
  simulate-differential-road-cbf-live \
  --config configs/experiments/dynamic_road_differential_v2.yaml \
  --output-dir /absolute/new/result/directory \
  --seed 7321609

python -m cbf_rover.entrypoints.research \
  simulate-differential-road-cbf-batch \
  --batch-config configs/experiments/dynamic_road_differential_multiseed.yaml \
  --catalog validation \
  --output-dir /absolute/new/result/directory
```

The exact source snapshot passed discovery `256/256` and held-out validation
`1024/1024`, followed by an independent byte-identical metrics replay. The
worst held-out seed also produced a 1× live animation with the inflated
obstacle boundaries, rover body and lookahead trajectories, live barrier
margin, `[v, omega]`, and intervention state. See the
[`single-seed run`](docs/experiments/DYNAMIC_ROAD_DIFFERENTIAL_RUN_2026-07-31.md)
and
[`multi-seed run`](docs/experiments/DYNAMIC_ROAD_DIFFERENTIAL_MULTISEED_RUN_2026-07-31.md).
These are empirical explicit-Euler exact-model results, not a q-SBC probability
proof and not a ROS2/Gazebo result.

### Gazebo Classic physical closed loop

The accepted server path now drives a physical differential rover model through
the same seeded road in Gazebo Classic 11 ODE. The online CBF is reevaluated
from the measured `[x, y, theta]` pose every `0.05 s`; all command bounds remain
inside the action projection and the adapter applies no post-CBF clipping.

- active continuous-camera recording contract:
  [`dynamic_road_gazebo_classic_v15.yaml`](configs/experiments/dynamic_road_gazebo_classic_v15.yaml);
- prior timestamp-aligned indoor-camera display contract:
  [`dynamic_road_gazebo_classic_v14.yaml`](configs/experiments/dynamic_road_gazebo_classic_v14.yaml);
- accepted 32-pattern control contract:
  [`dynamic_road_gazebo_classic_v13.yaml`](configs/experiments/dynamic_road_gazebo_classic_v13.yaml);
- frozen 32-pattern multi-seed contract:
  [`dynamic_road_gazebo_classic_multiseed_v4.yaml`](configs/experiments/dynamic_road_gazebo_classic_multiseed_v4.yaml);
- independent replay contract:
  [`dynamic_road_gazebo_classic_replay_v8.yaml`](configs/experiments/dynamic_road_gazebo_classic_replay_v8.yaml);
- mathematical and claim contract:
  [`DYNAMIC_ROAD_GAZEBO_CLASSIC_CONTRACT.md`](docs/mathematics/DYNAMIC_ROAD_GAZEBO_CLASSIC_CONTRACT.md);
- accepted server evidence:
  [`DYNAMIC_ROAD_GAZEBO_MULTISEED_V13_RUN_2026-07-31.md`](docs/experiments/DYNAMIC_ROAD_GAZEBO_MULTISEED_V13_RUN_2026-07-31.md),
  [`INDOOR_ROOM_CAMERA_CERTIFICATE_V14_RUN_2026-08-01.md`](docs/experiments/INDOOR_ROOM_CAMERA_CERTIFICATE_V14_RUN_2026-08-01.md), and
  [`INDOOR_ROOM_CAMERA_CERTIFICATE_VIDEO_V15_RUN_2026-08-01.md`](docs/experiments/INDOOR_ROOM_CAMERA_CERTIFICATE_VIDEO_V15_RUN_2026-08-01.md).

Server commands:

```bash
cmake -S simulation/gazebo_classic_bridge \
  -B build/gazebo_classic_bridge
cmake --build build/gazebo_classic_bridge --parallel

PYTHONPATH=src python -m cbf_rover.entrypoints.research \
  simulate-gazebo-classic-road-cbf-live \
  --config configs/experiments/dynamic_road_gazebo_classic_v15.yaml \
  --bridge-binary \
  build/gazebo_classic_bridge/cbf_rover_gazebo_classic_bridge \
  --output-dir /absolute/new/video-result/directory \
  --no-gif

PYTHONPATH=src python -m cbf_rover.entrypoints.research \
  simulate-gazebo-classic-road-cbf-live \
  --config configs/experiments/dynamic_road_gazebo_classic_v14.yaml \
  --bridge-binary \
  build/gazebo_classic_bridge/cbf_rover_gazebo_classic_bridge \
  --output-dir /absolute/new/result/directory \
  --no-gif

PYTHONPATH=src python -m cbf_rover.entrypoints.research \
  simulate-gazebo-classic-road-cbf-batch \
  --batch-config \
  configs/experiments/dynamic_road_gazebo_classic_multiseed_v4.yaml \
  --catalog validation \
  --bridge-binary \
  build/gazebo_classic_bridge/cbf_rover_gazebo_classic_bridge \
  --output-dir /absolute/new/batch-result/directory

PYTHONPATH=src python -m cbf_rover.entrypoints.research \
  validate-gazebo-classic-road-replay \
  --config \
  configs/experiments/dynamic_road_gazebo_classic_replay_v8.yaml \
  --run-manifest /absolute/first/run_manifest.json \
  --run-manifest /absolute/replay/run_manifest.json \
  --output-dir /absolute/new/validation/directory

PYTHONPATH=src python -m cbf_rover.entrypoints.research \
  render-gazebo-classic-road-cbf-run \
  --config configs/experiments/dynamic_road_gazebo_classic_v13.yaml \
  --run-manifest /absolute/accepted/run_manifest.json \
  --output-dir /absolute/new/render/directory
```

V15 uses loopback dashboard port `11571`; V14 retains `11471` for its frozen
historical configuration.

V14 binds the dashboard to server loopback port `11471`. View it during the
run through an SSH tunnel:

```bash
ssh -L 11471:127.0.0.1:11471 ai-server
```

Then open `http://127.0.0.1:11471/` locally. The HTTP server stops after the
terminal snapshot; `live_display/index.html`, `snapshot.json` and
`display_events.jsonl` remain in the result directory.

V14 preserves V13's controller, online distance-CBF, straight two-lane safety
set, and empirical `0.016 m / 0.065 rad` tracking envelope. It adds an indoor
room shell, an actual Gazebo/Ogre 960x540 front RGB sensor, and a camera-first
observer dashboard. Each atomic BMP embeds its Gazebo simulation timestamp;
the amber lifted-certificate boundary is projected with the nearest 0.05 s
measured rover pose. The main camera occupies most of the page, the top-down
map is secondary, only three metrics remain permanently visible, and detailed
telemetry is collapsed by default.

The final seed `7320014` run reached the goal at `39.25 s`, passed all 26
gates, recorded zero collisions and road departures, aligned `193/193` camera
updates with at most `0.024 s` error, covered `98.87%` of the episode, and
archived one timestamp-aligned certificate keyframe for each of five obstacle
activations. See the
[`V14 indoor-camera report`](docs/experiments/INDOOR_ROOM_CAMERA_CERTIFICATE_V14_RUN_2026-08-01.md).

The room is a visual and collision shell outside the frozen straight-road
domain; V14 is not a ring-track model. The camera and dashboard never feed the
controller. The action projection still uses the empirical lifted online
distance CBF, while the fixed-geometry q-SBC remains `not_ready`.

V13's historical dashboard remains on port `11469`; its 32-pattern discovery,
held-out validation, and replay evidence are not replaced by this single-seed
V14 visualization run.

V15 adds a lossless per-callback camera queue and continuous timestamp-aligned
recording. The final attempt2 reached the goal in `39.2 s` with `784`
transitions, activated `5/5` obstacles, and recorded zero collisions and road
departures. It archived contiguous camera counters `1..410` with zero missing
or invalid frames, `99.64%` temporal coverage, and at most `0.025 s` measured-
state alignment error. The formal report is
[`V15 camera-video run`](docs/experiments/INDOOR_ROOM_CAMERA_CERTIFICATE_VIDEO_V15_RUN_2026-08-01.md).
Its completed timestamp-resampled delivery is a 1920x1080 H.264 MP4 with `392`
frames at `10 fps`, exactly `39.2 s`. All five certificate overlays remain
visible, full GStreamer and OpenCV decoding pass, and the final frame displays
`COMPLETED / goal_reached`. The earlier 410-frame, 41.0-second callback-count
encoding remains diagnostic evidence and is not the final 1x artifact.

V8 retains V7's observer-only loopback dashboard and removes the low-frequency
`world_stats` topic from the per-step synchronization path. The companion
plugin returns one measured pose carrying both Gazebo simulation time and the
exact physics iteration; the bridge waits directly for the target iteration.
The controller, online CBF, `dt=0.05 s`, 50 physics steps per command and ODE
solver settings are unchanged.

Both independent V8 runs reached the goal with all five random obstacles, zero
collisions and zero road departures, and passed all 20 physical plus
live-display gates. Replay-v4 passed all 19 bounded-repeatability gates. The
final run achieved a real-time factor of `0.99991`, versus about `0.25` in V7,
and an external observer saw 176 distinct dashboard snapshots while the
controller was alive. The result is empirical and does not claim q-SBC, ROS2
or real-vehicle safety.

V13 then replaced the earlier conservative tracking support with the complete
V12-discovery-calibrated `0.016 m / 0.065 rad` empirical envelope and added a
hashed startup-attempt record. Up to three attempts are allowed only before an
initial state exists; a control trajectory is never retried. V4 discovery and
held-out validation both passed `32/32` lane patterns, replay-v8 passed all 19
bounded-repeatability gates, and every episode completed the observer-only
runtime certificate display. See the
[`V13 multi-seed report`](docs/experiments/DYNAMIC_ROAD_GAZEBO_MULTISEED_V13_RUN_2026-07-31.md).

> 2026-07-30 服务器正式运行状态：代码门禁通过，V1 选择 seed 11，但 PB1
> 三个既定稀疏多项式阶段均未通过独立 selection 门禁；研究链按合同停止为
> `not_ready_at_PB1_PAC`，未生成 controller 或 q-SBC 证书。完整证据见
> [`ROVER_FIRST_SERVER_RUN_2026-07-30.md`](docs/experiments/ROVER_FIRST_SERVER_RUN_2026-07-30.md)。
> 后续 DX1 已确认全局 PAC 结构失配；固定 C2 三分段低路由 controller 已通过
> C2V1、1000 条 exact rollout 和独立 replay。G5 的 W4/W6 均在第一目标返回
> `Unknown`，当前状态为 `controller_validated_qsbc_not_ready_stage1_unknown`。
> 证据见
> [`C2_QSBC_SERVER_RUN_2026-07-30.md`](docs/experiments/C2_QSBC_SERVER_RUN_2026-07-30.md)。

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
- [`static_obstacles.yaml`](configs/scenarios/static_obstacles.yaml) 已冻结 P1/D1
  的 exact、Gazebo、RViz 与指标共用集合真源。
- M1/S1/N1 exact model、ST1/V1 SAC、PB1/PA1/PS1 PAC、CV1 全域 controller
  验证、G4 q-SBC artifact 以及 G5 SOS/独立 replay 已进入源码。
- [`RESEARCH_CONTRACT.md`](docs/mathematics/RESEARCH_CONTRACT.md) 已按推荐项
  统一授权冻结，并由
  [`C2_PIECEWISE_CONTROLLER_CONTRACT.md`](docs/mathematics/C2_PIECEWISE_CONTROLLER_CONTRACT.md)
  固定 PB1 失败后的活动 controller 路线；C2 controller 已验证，但当前仍没有
  通过独立验证的 q-SBC artifact。
- Gazebo V13 源码已在服务器通过 Ruff、114 个快速测试、3 个
  MOSEK/SOS 慢测试以及 2 个 bridge 集成回归；差速多种子 discovery
  `256/256`、held-out validation
  `1024/1024` 及其独立 replay 均无碰撞、道路越界、超时、投影不可行或模型
  合同违例。
- Gazebo Classic V6 的两次独立物理闭环均到达终点并通过 15/15 单次门禁；
  replay-v2 的 19/19 门禁为 `accepted`，0 碰撞、0 越界，且可视化从哈希
  校验后的 Gazebo 测量轨迹生成。
- Gazebo Classic V8 在 V7 只读 dashboard 边界上增加了直接姿态 iteration
  同步；两次独立运行均通过 20/20 物理与显示门禁，209 次显示更新覆盖
  5 个障碍激活，replay-v4 通过 19/19 门禁，最终实时因子为 `0.99991`。
- Gazebo Classic V13 的 V4 discovery 与 held-out validation 均通过 `32/32`
  车道模式；validation 的碰撞、越界和超时均为 0，最大位置/航向残差为
  `0.012985 m / 0.046319 rad`，最小提升证书裕量为 `0.173300 m`。
  replay-v8 通过 19/19 门禁，全部控制轨迹在初始状态后均只执行一次。
- single-integrator 与 differential-drive 动态道路在线 CBF 均已通过各自的
  多种子经验门禁；固定几何 q-SBC 主线仍保持 `not_ready`，两者不能互相替代。
- learned RGB-D 的模型槽位仍为 `missing`，并继续作为默认禁用的经验基线。
- 本轮没有在本机运行测试、Python 入口、模型、ROS2、Colcon 或 Gazebo。

因此，当前源码状态是 `implementation_server_validated`，动态道路在线 CBF
在 exact 模型和 Gazebo Classic 物理闭环中的状态分别是
`empirically_accepted_exact_model` 与
`empirically_accepted_gazebo_classic`。固定几何 controller 状态是
`validated`，q-SBC 证书状态仍是 `not_ready_at_qSBC_stage1_unknown`。不能把
C2V1、在线 CBF、Gazebo 或 rollout 成功扩大解释为 reach-avoid 概率证明。

## 目录

```text
CBF-Rover/
├── src/cbf_rover/
│   ├── domain/                     # 纯数学/领域类型与端口
│   ├── methods/
│   │   ├── controllers/            # nominal controller 实现
│   │   ├── polynomial_sbc/         # PAC、全域验证、q-SBC 与 SOS/replay
│   │   └── baselines/learned_rgbd/ # 可选经验基线
│   ├── application/                # episode、编排、指标与 telemetry
│   ├── adapters/
│   │   ├── datasets/               # 数据格式适配
│   │   ├── storage/                # 记录存储适配
│   │   ├── exact/                  # M1/M2 exact simulator / Gym adapter
│   │   ├── gazebo/                 # Gazebo 测量状态、物理闭环与 replay
│   │   └── training/               # ST1/V1 与 learned baseline 训练适配
│   └── entrypoints/                # 研究 CLI 与 baseline CLI
├── simulation/ros2_ws/src/
│   ├── cbf_rover_description/      # 无仿真插件的机器人描述
│   ├── cbf_rover_sim/              # Gazebo world、插件与场景投影入口
│   └── cbf_rover_runtime/          # ROS2 运行适配边界
├── simulation/gazebo_classic_bridge/ # 无 ROS 的 Gazebo Classic transport 桥
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
| Rover-first 研究组合 | `configs/experiments/rover_first_qsbc.yaml` |
| 动态道路与差速在线 CBF | `configs/scenarios/dynamic_road_random_obstacles.yaml`、`configs/experiments/dynamic_road_*.yaml` |
| 服务器与 ROS2 | `configs/runtimes/server_ros2.yaml` |
| learned 对照组合与记录字段 | `configs/experiments/learned_rgbd_static_obstacles.yaml` |
| 数学证书制品契约 | `artifacts/certificates/schema.yaml` |
| learned 模型身份与来源 | `artifacts/models/manifest.yaml` |

`ScenarioSpec` 必须投影到 exact simulator、Gazebo 障碍生成、RViz marker、距离/
碰撞指标和运行快照。禁止在 launch、world、节点默认值或第二份 YAML 中重新
维护同一障碍几何。

本地模型 payload 统一放在项目内、Git 忽略的 `artifacts/payloads/`，并令
`CBF_ROVER_ARTIFACT_ROOT` 指向该目录；本地运行结果统一写入项目内、Git 忽略的
`runs/`，并令 `CBF_ROVER_RUN_ROOT` 指向该目录。远程服务器可使用独立的临时运行根，
但需要把正式交付物同步回本项目的 `runs/`。模型身份与加载验证不等于控制安全证明。

## 研究顺序

1. 保留已冻结的 ST1/V1/PB1、DX1 和所有失败诊断证据，不重训或复用失败系数。
2. 使用已通过 C2V1 分段闭包门禁和 1000 条 rollout 的冻结 C2。
3. 证书综合依次保留 G5/G6/G7/G8 结果；当前可运行制品是独立验证但
   `not_ready` 的 G7 W14，运行时只加载和评估，不修改 action。
4. M1 exact 已能输出 20 Hz 身份绑定遥测和 1×动画；详见
   [`实时仿真记录`](docs/experiments/EXACT_LIVE_CERTIFICATE_RUN_2026-07-30.md)。
5. 动态道路差速 exact 模型已完成多 seed 门禁和最坏 seed 1×实时显示；继续把
   该结果作为 Gazebo 跟踪误差建模前的经验基线。
6. 用同一个 seeded scenario 投影物理 Gazebo world、metrics 与只读 live
   dashboard；当前 V13 已在 Gazebo Classic 11 完成两套 32-pattern 经验物理
   闭环、运行时证书显示和有界 replay。跟踪包络不是形式证明，ROS2/RViz
   仍是独立、未验证的后续投影。
7. 最后才恢复 learned RGB-D 作为同场景、同 seed 的经验对照。

exact 仿真用于检查实现和实验行为，本身不会产生形式证明。差速车 Gazebo 结果
若没有经过证明的跟踪误差界，只能作为经验或诊断证据。

## 阅读顺序

1. [架构与依赖方向](docs/architecture/OVERVIEW.md)
2. [首个 Rover-first q-SBC 正式研究合同](docs/mathematics/RESEARCH_CONTRACT.md)
3. [实施与服务器运行计划](docs/mathematics/ROVER_FIRST_PLAN.md)
4. [证据与声明等级](docs/mathematics/CLAIM_LEVELS.md)
5. [math-first 架构 ADR](docs/adr/0001-math-first-five-layer-architecture.md)
6. [ScenarioSpec 与实验协议](docs/experiments/PROTOCOL.md)
7. [learned RGB-D 经验基线](docs/baselines/LEARNED_RGBD.md)
8. [服务器验证门禁](docs/experiments/SERVER_VALIDATION.md)
9. [本轮迁移与来源记录](docs/provenance/MATH_FIRST_REBUILD_2026-07-28.md)

任何静态文件存在、模型成功加载、测试通过、ROS2 构建成功或仿真冒烟，都只能
支持其实际覆盖的结论。声明等级必须由数学假设、兼容哈希、求解证据和独立验证
共同决定。
