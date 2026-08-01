# 2026-08-01 Gazebo V14 室内房间、前视相机与障碍证书显示

## Material Passport

- Origin Skill: experiment-agent
- Origin Mode: run
- Origin Date: 2026-08-01
- Verification Status: UNVERIFIED
- Version Label: indoor_room_camera_certificate_v14
- Final empirical run status: `provisional_success`

`UNVERIFIED` 限制的是研究结论的外推范围；`provisional_success` 表示这一次
服务器运行通过了预先绑定的经验门禁。两者不冲突。

## 1. 结论

V14 已完成用户要求的第一片：小车在室内双车道房间中运行，Gazebo/Ogre
实际前视 RGB 相机持续输出画面，随机障碍物依次激活，活动障碍的 lifted
online distance-CBF 边界以琥珀虚线和 `Oi h=...` 标签投影到相机画面。

Dashboard 采用相机主视图、俯视安全地图副视图、三个常驻指标和默认折叠的
完整遥测。参数不会覆盖轨道。相机和 dashboard 均为 observer-only；它们不
修改动作，也不向 controller 回传数据。

最终 seed `7320014` 运行到达终点，通过 26/26 门禁，碰撞和道路越界均为 0。
相机 `193/193` 个发布帧都解析了嵌入式 Gazebo 仿真时间戳并匹配到最近的
`0.05 s` 实测状态；最大时间误差为 `0.024 s`。五个障碍各保留一张“该障碍
已在对齐状态中激活”的相机关键帧和配对 snapshot。

本结果属于单 seed 的经验 Gazebo Classic 11 ODE 物理闭环。它不是 q-SBC
概率证明，不是 ROS2 结果，不是图像感知安全闭环，也不是实车安全证明。

## 2. 为什么本片采用房间而不是环形跑道

V13 冻结的道路集合、nominal controller、在线 CBF 道路边界和 32-pattern
经验验证都基于直线双车道。直接弯成环形跑道会同时改变集合几何、控制器和
证书约束，不能只作为视觉替换。

因此 V14 选择室内房间外壳：墙体位于冻结直道路域之外，保持控制数学不变，
先验证相机、实时显示和证书融合。环形跑道属于后续新的曲线坐标合同，不能把
本报告解释为已经完成。

## 3. 未改变与新增的合同

未改变：

- controller、lookahead 映射与 online robust distance-CBF；
- `dt=0.05 s`、`0 <= v <= 0.5 m/s`、`|omega| <= 1.5 rad/s`；
- 不允许受证动作后置 clipping；
- V13 的 `0.016 m / 0.065 rad` 经验跟踪包络；
- 直线双车道安全域和 seed `7320014` 的障碍生成规则。

V14 新增：

- 室内房间视觉/碰撞外壳；
- 960x540、10 Hz、向下俯视 `0.14 rad` 的 Gazebo CameraSensor；
- Xvfb 无界面渲染；
- 在同一个原子 BMP 中嵌入 Gazebo 秒、纳秒和单调帧号；
- 相机帧到最近实测状态的时间对齐与误差门禁；
- 激活后相机关键帧门禁；
- camera-first、telemetry-collapsed 的 observer dashboard。

数据流：

```text
measured Gazebo pose ---> online lifted distance CBF ---> certified [v, omega]
         |                            |
         |                            +----> observer certificate frame
         v
Gazebo/Ogre CameraSensor ---> atomic BMP + embedded sim timestamp
         |                            |
         +----------> nearest measured-state alignment
                                      |
                                      v
                  camera PNG + amber certificate projection
                                      |
                                      v
                  loopback dashboard (no control feedback)
```

## 4. 源码、配置与二进制身份

| 项目 | 身份 |
|---|---|
| 服务器源码目录 | `/data/users/csynth/cbf_rover_runs/source_indoor_camera_v14_candidate_6ce73336786a` |
| 服务器构建目录 | `/data/users/csynth/cbf_rover_runs/build_indoor_camera_v14_candidate_6ce73336786a` |
| V14 配置 SHA-256 | `710829f95b9eb4724da5e774db39909bd4fba9559f3a24683cdef303b6a8e3b7` |
| bridge SHA-256 | `a786944da7fd169dbca6c2c46116004d2caa004fe8d80704aced408010cbbf7e` |
| state plugin SHA-256 | `888a8e4b746adec1090886128c7a98a23fd574f3174e4c6e10d9e4805efb5f1b` |
| camera plugin SHA-256 | `3677b3d100534ccb1165d609aa440b399da2c033981ddf06c54241e5165f379a` |
| 可执行关键文件数 | `199` |
| 本地/服务器可执行清单 SHA-256 | `5f024a26c54f87eb896a67d59c6c880b4e4d2fa85421f1d91b245ff20d58457b` |

服务器目录名保留初始 candidate 后缀以连续保存失败历史，不能把目录名中的
`6ce733...` 当作最终源码哈希。最终身份以文件级哈希和上述 199 文件清单为准。

## 5. 服务器命令

工作目录：

```text
/data/users/csynth/cbf_rover_runs/source_indoor_camera_v14_candidate_6ce73336786a
```

构建：

```bash
cmake --build \
  /data/users/csynth/cbf_rover_runs/build_indoor_camera_v14_candidate_6ce73336786a \
  --parallel 2
```

单元与配置门禁：

```bash
PYTHONPATH=src python -m pytest -q \
  tests/unit/adapters/test_gazebo_classic_road.py \
  tests/unit/adapters/test_gazebo_live_display.py \
  tests/unit/test_research_configs.py
```

相机集成探针：

```bash
CBF_ROVER_GAZEBO_BRIDGE_BINARY=/absolute/build/cbf_rover_gazebo_classic_bridge \
GAZEBO_PLUGIN_PATH=/absolute/build \
PYTHONPATH=src python -m pytest -q \
  tests/ros2/test_gazebo_classic_bridge.py::test_v14_front_camera_writes_atomic_bmp_frames
```

最终 episode；外层 timeout 为 180 s，配置自动启用 1x wall-clock pacing：

```bash
GAZEBO_PLUGIN_PATH=/absolute/build PYTHONPATH=src timeout 180s python -m \
  cbf_rover.entrypoints.research simulate-gazebo-classic-road-cbf-live \
  --config configs/experiments/dynamic_road_gazebo_classic_v14.yaml \
  --bridge-binary /absolute/build/cbf_rover_gazebo_classic_bridge \
  --output-dir /absolute/new/episode_seed_7320014_attempt4 \
  --seed 7320014 --no-gif
```

运行时远程查看：

```bash
ssh -L 11471:127.0.0.1:11471 ai-server
```

随后在本机打开 `http://127.0.0.1:11471/`。HTTP 服务随 episode 结束，页面、
snapshot 和事件流继续保留为结果制品。

## 6. 最终服务器结果

结果目录：

```text
/data/users/csynth/cbf_rover_runs/results_indoor_camera_v14_candidate_6ce73336786a/episode_seed_7320014_attempt4
```

| 指标 | 结果 |
|---|---:|
| status / termination | `provisional_success / goal_reached` |
| acceptance gates | `26 / 26` |
| transitions / frames | `785 / 786` |
| simulated / wall duration | `39.25 / 39.2956 s` |
| achieved realtime factor | `0.99884` |
| activated obstacles | `5 / 5` |
| collision / road departure | `0 / 0` |
| filter intervention steps | `130` |
| minimum lifted certificate barrier | `0.1791713794 m` |
| minimum CBF constraint residual | `-6.25e-17 m/s` |
| minimum measured contraction residual | `0.0346121075 m` |
| maximum position tracking residual | `0.0126918719 m` |
| maximum heading tracking residual | `0.0452453717 rad` |
| maximum roll / pitch | `3.40e-5 / 0.00631629 rad` |
| post-projection command clipping | none |
| live display updates | `198` |

关键制品：

- `run_manifest.json` SHA-256:
  `902fc1e5eb201a8a8bb0fd26858355123fe2a049baa4ff5273d0290163aa3d57`；
- `live_trace.jsonl` SHA-256:
  `31dd25c190465ab86d1ab4a74c5ce221981703396cffc5f166b8dac374816410`；
- `live_display_manifest.json` SHA-256:
  `469ba8ae8ad9a8af4c5e00fd2c36109c87831a4956ab68be9546e568617fc006`。

结果目录中没有遗留 `.tmp` 文件。

## 7. 相机时间对齐与激活关键帧

| 指标 | 结果 |
|---|---:|
| camera updates / aligned updates | `193 / 193` |
| invalid frames | `0` |
| capture coverage | `0.9887388535` |
| maximum alignment error | `0.024 s` |
| pending activation keyframes | `0` |

| 障碍 | controller 激活 step | 归档 step | 相机对齐 step | 对齐误差 |
|---|---:|---:|---:|---:|
| O1 | 9 | 12 | 10 | `0.001 s` |
| O2 | 161 | 164 | 161 | `0.001 s` |
| O3 | 253 | 256 | 255 | `0.004 s` |
| O4 | 364 | 368 | 367 | `0.001 s` |
| O5 | 474 | 476 | 475 | `0.001 s` |

每一行的 aligned active IDs 都已经包含对应障碍；门禁同时检查五个障碍无重复、
关键帧与 snapshot 哈希存在、对齐记录存在且没有等待项。

## 8. Dashboard 浏览器 QA

最终 QA 使用 attempt4 的真实 O3 Gazebo 关键帧和配对的时间对齐 snapshot，
在服务器 Firefox headless 中分别以 1440x1080 和 430x932 渲染。Firefox CLI
会在页面异步 fetch 完成前截图，因此 QA harness 增加一个隐藏的 2 s 延迟资源；
生产 viewer 的 JavaScript、布局和投影逻辑未修改。该 QA 是归档真实帧的浏览器
重放，不冒充运行中现场截图。

目检结果：

- desktop 和 mobile 都处于 `RUNNING`，时间 `12.80 s`，活动障碍 `3/5`；
- 前视画面可见房间、车道、红色障碍、琥珀虚线和 `O03 h=2.219m`；
- 俯视地图同步显示轨迹、障碍和证书圆；
- 只有三个关键指标常驻，完整遥测折叠，未覆盖相机轨道。

| 制品 | SHA-256 |
|---|---|
| desktop screenshot | `f2bfad7776373519bf94c89fbf33d9eeeec7ecde7db5b19cb6a1f9facd4884e6` |
| mobile screenshot | `b2e977a0afa42abeb5ef3ac2f5afede8d8e149a7774194ba93e873c51a09562a` |
| QA manifest | `fff747cc1b9f618f6aa9120881dcb7e2b5cd2cd35069c58d05446aabcdcd729e` |

实时更新本身由 198 次 display update、193 次 timestamped camera update 和
`display_events.jsonl` 支撑；浏览器截图只验证最终显示布局和 Canvas 投影。

## 9. 保留的失败与修复历史

| 尝试 | 结果 | 处理 |
|---|---|---|
| 初始 camera probe 1/2 | 在写下一帧时观察到私有 `.tmp`，过度严格失败 | 改为验证原子发布目标；停机后的精确清理由正式运行器负责 |
| full attempt1 | 动力学通过，但相机终帧主要是天空/地面，dashboard 截到 waiting 状态 | 相机下俯 `0.14 rad`，保存障碍激活关键帧，修复 image load handler |
| full attempt2 | 相机、关键帧和浏览器布局通过，但 BMP 无仿真时间戳 | 在同一 BMP 内嵌时间戳/帧号，加入最近实测状态对齐 |
| 首次 dashboard QA | Python quoting `SyntaxError`，未生成有效 HTML | 原文件保留且不作为证据；使用新目录重跑 |
| 早期 Firefox QA | desktop 抓在 snapshot 前，mobile 抓在第二次 image load 前 | QA-only 隐藏延迟资源，肉眼确认琥珀投影与标签 |
| timestamp probe 1 | 突发推进到 0.301 s 时相机仅处理到 0.137 s | 探针按正式运行的 0.05 s 墙钟节拍推进并等待新鲜帧 |
| timestamp probe 2 | 时间戳已通过，但错误要求探针强退后无 `.tmp` | 移除混淆生命周期的断言；probe 3 通过 |
| full attempt3 | 26/26 门禁通过，但 O2/O4 归档图来自激活前一帧 | 延迟归档至对齐状态包含新障碍，并把该条件加入门禁 |
| full attempt4 | `provisional_success`，26/26 门禁通过 | 当前最终证据 |
| build hash attempt1 | 编译成功，但哈希命令用了错误的 bridge 别名而退出 1 | 正确文件名复核三项二进制哈希；失败日志保留 |

任何失败结果都没有被覆盖或删除。

## 10. 服务器测试门禁

| 测试 | 最终结果 | 日志 SHA-256 |
|---|---:|---|
| V14 unit/config suite | `24 passed in 3.94 s` | `7a4a763251c0455f00ab88bbe93a143d3f0bd6a47aaa13266be8b1b35561c66f` |
| timestamped camera integration probe | `1 passed in 5.21 s` | `6e8a33e054bd3f39cae75b9fda29a5ae1c807adde759ef9b1b64b5659fd20d5c` |
| final full simulation command | exit `0` | `b3ba689eb9f03b30027ed4445edddc669fd9d375d7d5dec6ea5eee799a8c074a` |

本机没有运行 Python、Gazebo、ROS2 或仿真测试，只做源码编辑、哈希复核和下载后
图像目检。

## 11. 制品位置

本地下载证据：

```text
E:\6_Engineering_Projects\CBF-Rover\runs\indoor_camera_v14_seed_7320014_attempt4
```

该目录包含 run manifest、完整 trace、world、obstacle plan、五组关键帧及
snapshot、最终 desktop/mobile QA、QA manifest 和服务器构建/测试/失败日志。

## 12. 原始接手目录复核

最终源码快照明确排除 `接手的原始文件/`。本机只读复核仍只有三份接手压缩包：

| File | Bytes | SHA-256 |
|---|---:|---|
| `rgbd_safe_method.7z` | 42,052 | `ab2f0a0eceb97d78cb825e04adeac2beb64d27cd3f53a937ff4475708e3893b0` |
| `yzh-skills.7z` | 44,717,933 | `72de126de7a22ef0e6e6b82838710d2dd169822620e1b27fb32d0f886e2eca21` |
| `yzh1-杨子恒的所有原始代码.7z` | 860,136,841 | `405c1a810d72915f14ba759d282e79cc1d8c2d585ee787a591be029e0fe66a07` |

文件数、大小和逐文件哈希均与既有基线一致；本轮没有在该目录中解压、新增、
删除、改名或修改任何内容。

## 13. 声明边界

- action projection 使用经验 lifted online robust distance CBF；
- 相机琥珀线是同一证书在时间对齐实测位姿下的几何投影，不是新综合证书；
- 图像不参与障碍检测、controller 或 CBF，不得称为视觉感知安全闭环；
- V13 跟踪包络是 Gazebo 经验包络，不是形式误差界；
- 本片只有一个 seed，不是新的多 seed 泛化或任意随机几何结论；
- 固定几何 q-SBC 仍为
  `controller_validated_qsbc_not_ready_stage1_unknown / not_ready`；
- 不声明概率 reach-avoid、ROS2、RViz、真实摄像头或实车安全。
