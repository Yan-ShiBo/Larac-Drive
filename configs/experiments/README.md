# Experiments

实验配置只负责组合 system、controller、certificate、scenario、runtime 或
baseline，并定义对照关系和记录字段。领域参数必须留在各自真源中。

当前配置分为三条互不混淆的研究链：

- `rover_first_qsbc.yaml`：冻结 P1/D1、controller 选择与固定几何 q-SBC
  综合合同；当前证书状态仍为 `not_ready`。
- `dynamic_road_multiseed.yaml`：二维 single-integrator 动态道路在线 CBF
  多种子门禁。
- `dynamic_road_differential_v3.yaml` 与
  `dynamic_road_differential_multiseed_v2.yaml`：当前差速运动学 lookahead
  在线 CBF 单次实时显示和多种子门禁。较早版本保留投影不可行、修复和回归
  证据，不作为当前运行入口。
- `dynamic_road_gazebo_classic_v13.yaml`：已接受的 Gazebo Classic 多 seed
  物理差速闭环合同。它冻结 V12 完整 discovery 校准得到的
  `0.016 m / 0.065 rad` 经验跟踪包络，并将最多三次启动尝试严格限制在
  取得初始状态之前。V1–V12 保存失败、校准、时间戳对齐、只读显示和启动
  诊断证据，不得覆盖。
- `dynamic_road_gazebo_classic_v14.yaml`：当前室内房间、实际 Gazebo 前视
  RGB 相机与 camera-first dashboard 的单 episode 可视化入口。它继承 V13
  的 controller、在线 distance CBF、直道路域与经验跟踪包络，只扩展视觉
  外壳、带仿真时间戳的原子相机帧和 observer-only 证书投影。相机时间戳与
  最近的 `0.05 s` 实测状态对齐；验收同时要求帧覆盖率、对齐误差和每个障碍
  激活后的关键帧。它不取代 V13 的 32-pattern 多 seed 与 replay 证据。
- `dynamic_road_gazebo_classic_v15.yaml`：在 V14 observer 边界上增加逐 camera
  callback 的无损文件队列、连续 timestamp-aligned PNG 录制和 1920x1080
  camera-primary 视频合同。seed `7320014` 的 attempt2 已完整归档 counter
  `1..410`、missing/invalid 均为 0，录制状态为 `completed`；最终 MP4 已按
  Gazebo capture timestamp 重采样为 `392` 帧、`10 fps`、`39.2 s`，并通过完整
  解码和 O1--O5 可见性检查。410 帧、41.0 秒的中间编码只保留为诊断制品。
  它仍是单 seed 经验结果，不取代 V13 多 seed 与 replay 证据。
- `dynamic_road_gazebo_classic_replay_v2.yaml`：要求两个独立 V6 结果通过
  身份、文件哈希、全部单次门禁和有界物理重复性验收；不要求 ODE 轨迹逐字节
  一致。
- `dynamic_road_gazebo_classic_replay_v3.yaml`：要求两个独立 V7 结果通过
  同一组有界物理重复性门禁；V7 单次验收同时要求实时显示完成、更新数正确、
  终止时间对齐、显示端无控制反馈并与配置一致。
- `dynamic_road_gazebo_classic_replay_v4.yaml`：在相同门禁上绑定两个独立
  V8 结果，验证“姿态时间戳 + physics iteration”直接同步不会改变物理验收，
  并移除每个控制步对低频 `world_stats` 的等待。
- `dynamic_road_gazebo_classic_replay_v5.yaml` 至
  `dynamic_road_gazebo_classic_replay_v8.yaml`：依次绑定 V9–V13；当前 V8
  同时校验两次运行各自的 `startup_attempts.json`，并保持有界经验重复性而
  不要求 ODE 轨迹逐字节一致。
- `dynamic_road_gazebo_classic_multiseed_v4.yaml`：当前 discovery/validation
  合同。两套目录各显式冻结 32 个 seed，逐一覆盖五个障碍的全部车道模式；
  每个 episode 保持 1×墙钟和只读实时证书显示。只允许控制开始前的有界、
  可审计启动尝试，控制开始后不得重跑。V1–V3 是冻结前序证据。
- `learned_rgbd_static_obstacles.yaml`：默认禁用的 learned RGB-D 经验对照。
- `pb1_failure_diagnostic.yaml` 与 `qsbc_expressivity_diagnostic.yaml`：保存
  失败诊断合同，不用于绕过既定研究门禁。

动态道路 exact 配置和 Gazebo 配置属于不同证据层级。前者是显式模型经验结果，
后者是 Gazebo Classic ODE 物理闭环经验结果；两者都不是 q-SBC 概率证明。
Gazebo V13、多 seed V4、V14 与 V15 也不是 q-SBC、ROS2 或实车结果。V14/V15 中
参与 action projection 的仍是经验 lifted online distance CBF；相机上的琥珀
边界只是同一运行时证书的时间对齐几何投影，图像不参与控制。Python、测试和
仿真只在服务器环境运行。
