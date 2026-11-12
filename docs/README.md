# Larac-Drive 文档索引

项目面向正常跟驰、前车急刹和邻车切入三类工况。横纵向学习控制和在线安全约束通过闭环仿真与基线对比评价。概率验证聚焦同车道前车随机制动下的三维纵向应急子系统。[项目方向、技术路线与验收目标](PROJECT_DIRECTION.md)

## 架构

- [车辆控制与概率验证的目标架构、已有五层代码结构](architecture/OVERVIEW.md)
- [ADR-0001：采用 math-first 五层架构](adr/0001-math-first-five-layer-architecture.md)

## CBF-Rover 阶段的数学合同与声明

- [首个 Rover-first q-SBC 正式研究合同](mathematics/RESEARCH_CONTRACT.md)
- [C2 分段多项式 controller 合同](mathematics/C2_PIECEWISE_CONTROLLER_CONTRACT.md)
- [G4 q-SBC 定理证明](mathematics/G4_QSBC_THEOREM_PROOF.md)
- [G6 局部坐标 SOS 数值合同](mathematics/G6_LOCAL_COORDINATE_SOS_CONTRACT.md)
- [G7 高阶 Bernstein LP 证书综合合同](mathematics/G7_BERNSTEIN_LP_CONTRACT.md)
- [G8 标量乘子 Bernstein 证书综合合同](mathematics/G8_SCALAR_MULTIPLIER_BERNSTEIN_CONTRACT.md)
- [G10 Chebyshev 多项式乘子 LP 合同](mathematics/G10_CHEBYSHEV_POLYNOMIAL_MULTIPLIER_CONTRACT.md)
- [G10R 同-box 冗余消除 LP 合同](mathematics/G10R_REDUNDANCY_REDUCED_LP_CONTRACT.md)
- [G10E 高阶冗余消除 LP 合同](mathematics/G10E_EXTENDED_REDUCED_LP_CONTRACT.md)
- [G10A V3 精确盒反馈主动 LP 合同](mathematics/G10A_ACTIVE_BERNSTEIN_FEEDBACK_CONTRACT.md)
- [G10A2 精确 Bernstein 预分离合同](mathematics/G10A2_EXACT_PRESEPARATION_CONTRACT.md)
- [G10A3 精确网格多点分离合同](mathematics/G10A3_EXACT_GRID_MULTIPOINT_CONTRACT.md)
- [G11P49 C0 分片 barrier 与 roof witness 合同](mathematics/G11P49_C0_PIECEWISE_ROOF_CONTRACT.md)
- [动态公路随机障碍在线 CBF 合同](mathematics/DYNAMIC_ROAD_ONLINE_CBF_CONTRACT.md)
- [动态公路多 seed 压力测试协议](mathematics/DYNAMIC_ROAD_MULTISEED_PROTOCOL.md)
- [动态公路差速运动学 CBF 合同](mathematics/DYNAMIC_ROAD_DIFFERENTIAL_CONTRACT.md)
- [动态公路差速运动学 CBF 合同 V2](mathematics/DYNAMIC_ROAD_DIFFERENTIAL_CONTRACT_V2.md)
- [动态公路差速运动学多 seed 协议](mathematics/DYNAMIC_ROAD_DIFFERENTIAL_MULTISEED_PROTOCOL.md)
- [动态公路 Gazebo Classic 测量状态 CBF 合同](mathematics/DYNAMIC_ROAD_GAZEBO_CLASSIC_CONTRACT.md)
- [动态公路 Gazebo Classic 多 seed 协议](mathematics/DYNAMIC_ROAD_GAZEBO_MULTISEED_PROTOCOL.md)
- [Rover-first q-SBC 研究实施计划](mathematics/ROVER_FIRST_PLAN.md)
- [证据等级、运行评估与声明边界](mathematics/CLAIM_LEVELS.md)
- [证书制品 schema](../artifacts/certificates/schema.yaml)

## CBF-Rover 阶段的基线与实验

- [learned RGB-D 经验基线](baselines/LEARNED_RGBD.md)
- [实验协议与 ScenarioSpec](experiments/PROTOCOL.md)
- [服务器验证门禁](experiments/SERVER_VALIDATION.md)
- [2026-07-30 Rover-first 正式服务器运行记录](experiments/ROVER_FIRST_SERVER_RUN_2026-07-30.md)
- [2026-07-30 C2 与 q-SBC 正式服务器运行记录](experiments/C2_QSBC_SERVER_RUN_2026-07-30.md)
- [2026-07-30 M1 exact 障碍证书实时仿真记录](experiments/EXACT_LIVE_CERTIFICATE_RUN_2026-07-30.md)
- [2026-07-30 动态公路随机障碍在线 CBF 仿真记录](experiments/DYNAMIC_ROAD_CBF_RUN_2026-07-30.md)
- [2026-07-31 动态公路多 seed 压力测试记录](experiments/DYNAMIC_ROAD_MULTISEED_RUN_2026-07-31.md)
- [2026-07-31 动态公路差速运动学 CBF 仿真记录](experiments/DYNAMIC_ROAD_DIFFERENTIAL_RUN_2026-07-31.md)
- [2026-07-31 差速运动学多 seed 压力测试记录](experiments/DYNAMIC_ROAD_DIFFERENTIAL_MULTISEED_RUN_2026-07-31.md)
- [2026-07-31 Gazebo Classic 物理闭环与实时证书记录](experiments/DYNAMIC_ROAD_GAZEBO_CLASSIC_RUN_2026-07-31.md)
- [2026-07-31 Gazebo 运行时障碍证书显示 V7](experiments/DYNAMIC_ROAD_GAZEBO_LIVE_DISPLAY_RUN_2026-07-31.md)
- [2026-07-31 Gazebo 实时障碍证书 V8 性能优化](experiments/DYNAMIC_ROAD_GAZEBO_REALTIME_V8_RUN_2026-07-31.md)
- [2026-07-31 Gazebo V13 全车道模式验证、replay 与证书渲染](experiments/DYNAMIC_ROAD_GAZEBO_MULTISEED_V13_RUN_2026-07-31.md)
- [2026-08-01 Gazebo V14 室内房间、前视相机与时间对齐证书显示](experiments/INDOOR_ROOM_CAMERA_CERTIFICATE_V14_RUN_2026-08-01.md)
- [2026-08-01 Gazebo V15 连续前视相机录制与障碍证书演示](experiments/INDOOR_ROOM_CAMERA_CERTIFICATE_VIDEO_V15_RUN_2026-08-01.md)
- [2026-08-01 Gazebo V16 稀疏障碍、实时世界模型、轨迹与 h=q_h 演示](experiments/INDOOR_SPARSE_REALTIME_CERTIFICATE_Q_LEVEL_V16_RUN_2026-08-01.md)

## 来源与制品

- [来源、制品与永久只读归档](provenance/README.md)
- [2026-07-28 math-first 整体重构](provenance/MATH_FIRST_REBUILD_2026-07-28.md)
- [前一阶段清洁重构历史记录](provenance/CLEAN_REBUILD_2026-07-28.md)
- [模型制品清单](../artifacts/models/manifest.yaml)
- [模型制品模式](../artifacts/models/schema.yaml)

仓库内不提交运行生成的证书系数，learned 模型槽位也全部为 `missing`。
2026-07-30，C2 已通过分段形式门禁、1000 条 exact rollout 与独立 replay；
G7 W14 在远程服务器结果根中产生了独立验证的
`formal_valid_not_ready` q-SBC，并完成 1×实时 M1 exact 仿真。G5/G6/G8 的
失败和 G9 诊断均保留为失败证据；其本地镜像统一位于项目内 `runs/`。

2026-07-31，动态道路 single-integrator 与 differential-drive 在线 CBF 均完成
服务器多 seed 门禁和最坏 seed 1×实时渲染。随后 Gazebo Classic 11 ODE
差速物理闭环完成两次独立 V5 运行并通过有界 replay 验收，状态为经验
`accepted`。它不改变 q-SBC 的 `not_ready` 状态，也不代表 ROS2 或实车验证。
V7 在时间戳对齐的 Gazebo 闭环上增加了只读运行时 dashboard，并通过 20/20
单次门禁与 replay-v3 的 19/19 门禁。
V8 进一步以同一条时间戳姿态携带的 physics iteration 直接完成逐步同步，
把完整墙钟实时因子从约 `0.25` 提高到 `0.99991`；两次运行均通过 20/20
门禁，replay-v4 通过 19/19 门禁。
V13 随后冻结 discovery 校准的 `0.016 m / 0.065 rad` 经验跟踪包络和只允许
控制前启动尝试的审计边界。V4 discovery 与未参与调参的 validation 均为
`32/32`，validation 的 21 个总门禁全部通过；最坏成员独立重复的 replay-v8
通过 19/19，最小证书裕量场景已生成 1× GIF 与终态图。结论仍只属于有限目录
Gazebo 经验验证，不改变 q-SBC `not_ready`、ROS2 未验证和无实车声明。
本机仍未运行 Python、测试、模型、ROS2、Colcon 或 Gazebo。
