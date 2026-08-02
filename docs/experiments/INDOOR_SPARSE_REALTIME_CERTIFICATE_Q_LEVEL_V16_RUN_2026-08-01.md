# 2026-08-01 Gazebo V16 稀疏障碍、实时世界模型、轨迹与 `h=q_h` 演示

## 结论

V16 已在服务器 Gazebo Classic 11 / ODE 中完成一次新的单 seed 物理闭环运行。
seed `7320014` 的 Rover 在室内直道场景中到达终点，3 个稀疏障碍全部激活，碰撞
和道路越界均为 0，运行状态为 `provisional_success`，全部 36 个单次验收门通过。

本片实现四个可观察目标：

- 将障碍数量降为 3，并在约 `4.1--4.3 m` 前揭示，路线控制器使用 `3.8 m`
  lookahead 提前开始绕行；
- 每个 `0.05 s` 控制步都从时间戳对齐的 Gazebo 实测位姿重建活动障碍世界模型，
  并精确求解二维 halfspaces 与速度圆盘交集上的 action projection；
- 在项目内保存包含全部 711 个控制状态的 `trajectory.csv`；
- 在实时 dashboard 和最终 MP4 中同时绘制琥珀色 `h=0` 安全边界与青色
  `h=q_h` 预览等值线，固定 `q_h=0.30 m`。

## 配置与运行位置

| 项目 | 路径或身份 |
|---|---|
| Gazebo config | `configs/experiments/dynamic_road_gazebo_classic_v16.yaml` |
| differential config | `configs/experiments/dynamic_road_differential_v4.yaml` |
| sparse scenario | `configs/scenarios/dynamic_road_sparse_obstacles.yaml` |
| server source | `/data/users/csynth/cbf_rover_runs/source_feature_v16_red_20260801` |
| server result | `/data/users/csynth/cbf_rover_runs/results_feature_v16_sparse_q_20260801/episode_seed_7320014_attempt3` |
| project-local delivery | `runs/indoor_sparse_realtime_q_v16_seed_7320014_attempt3` |

服务器运行命令：

```bash
cd /data/users/csynth/cbf_rover_runs/source_feature_v16_red_20260801
PYTHONPATH=src \
/data/users/csynth/cbf_rover_runs/results_20260729T184104Z_4525410e0fbc/environment/venv/bin/python \
  -m cbf_rover.entrypoints.research \
  simulate-gazebo-classic-road-cbf-live \
  --config configs/experiments/dynamic_road_gazebo_classic_v16.yaml \
  --bridge-binary \
  /data/users/csynth/cbf_rover_runs/build_indoor_camera_v15_queue_candidate_caff5554d9f4/cbf_rover_gazebo_classic_bridge \
  --output-dir \
  /data/users/csynth/cbf_rover_runs/results_feature_v16_sparse_q_20260801/episode_seed_7320014_attempt3 \
  --seed 7320014 --no-gif
```

命令退出码为 0。外层 attempt3 的第一次启动在控制开始前等待 bridge 超时；进程
关闭后，相机残留隔离器确认没有可归档帧，第二次启动取得初始状态并被选中。
`control_frames_before_selection=0`，因此正式轨迹没有跨启动 attempt 污染。V16 使用
隔离的 Gazebo master `127.0.0.1:11672` 和只绑定 loopback 的 dashboard
`127.0.0.1:11671`，没有操作服务器上其他 Gazebo 任务。

## 稀疏障碍与提前绕行

| 障碍 | 中心 x | 揭示距离 | 激活 step / time | 激活时前向距离 |
|---|---:|---:|---:|---:|
| O1 | `5.200 m` | `4.298 m` | `21 / 1.05 s` | `4.128 m` |
| O2 | `8.514 m` | `4.139 m` | `220 / 11.00 s` | `3.973 m` |
| O3 | `11.631 m` | `4.121 m` | `370 / 18.50 s` | `3.966 m` |

第一次路线换道和 CBF intervention 出现在 step `37`、`1.85 s`。此时 Rover
中心 `x=1.256 m`，O1 的 obstacle barrier 仍为 `3.138 m`，因此绕行并非到贴近
障碍时才开始。障碍的随机几何由 seed 冻结；未激活障碍不会进入控制器世界模型。

## 在线世界模型、求解与轨迹

运行产生 711 个状态帧和 710 个 transition，仿真时长 `35.5 s`。对
`live_trace.jsonl` 的逐行审计结果为：

- 711/711 帧的 `world_model.update_index == step_index`；
- 711/711 帧声明从时间戳对齐的 measured Gazebo pose 重建世界模型；
- 711/711 帧的 `certificate.online_solve.world_model_update_index == step_index`；
- 所有动作步使用 `exact_2d_projection_onto_halfspaces_intersect_l2_disk`，没有
  controller 后置 clipping；终止帧只记录终止状态；
- `q_h` 在全程保持 `0.30 m`，显示语义保持
  `runtime_distance_cbf_level_not_qsbc_probability`。

`trajectory.csv` 是 `live_trace.jsonl` 的全速率扁平化轨迹，包含 step、仿真时间、
Gazebo 时间、Rover pose、lookahead、目标车道、差速命令、过滤动作、最小 barrier、
活动障碍 ID、世界模型数量、求解状态、`q_h` 与终止原因。共 711 行数据。

| 物理与控制指标 | 结果 |
|---|---:|
| termination | `goal_reached` |
| activated obstacles | `3 / 3` |
| collisions / road departures | `0 / 0` |
| filter intervention steps | `13` |
| minimum lifted certificate barrier | `0.3027083106 m` |
| minimum active-obstacle barrier | `0.4502572487 m` |
| minimum CBF constraint residual | `0.5990980900 m/s` |
| minimum realized contraction residual | `0.0602420863 m` |
| maximum position tracking residual | `0.0126918719 m` |
| maximum heading tracking residual | `0.0452411778 rad` |
| achieved real-time factor | `0.9999059700` |

## 相机、等值线与视频

相机录制归档了 354 个连续 callback 帧，missing frame 为 0，覆盖率为
`0.9970704225`，最大源帧间隔为 `0.35 s`。354 个完整 `.bmp` 帧已全部消费，
关闭时 remaining frame、temporary file 和本次丢弃的未发布 writer buffer 都为 0。
生命周期清理只允许命中 `frame_*.bmp.tmp`，且丢弃数量必须不大于 1；非目标临时
文件会保留并令门禁失败。该 allowlist 与数量上限均由专门测试覆盖。

最终视频为 `camera_video/gazebo_front_camera_sparse_realtime_cbf_q_level_1x.mp4`：

| 视频指标 | 结果 |
|---|---:|
| status | `completed` |
| source / rendered frames | `354 / 355` |
| resolution / rate | `1920x1080 / 10 fps` |
| encoded duration | `35.5 s` |
| visible certificate output frames | `241` |
| display level | `h=q_h`, `q_h=0.30 m` |
| MP4 SHA-256 | `d6557b16369cdf13072924475ebbbfdbdc543b0077961a83b957e1839860155c` |

青色轮廓半径按
`certificate_radius_m + additional_body_inflation_m + q_h_m` 计算；琥珀色轮廓是
参与 action projection 的 `h=0` 边界。相机主画面占据左侧大区域，俯视轨迹图与
3 个精简指标位于右侧，不遮挡道路。

## 证据哈希

| 制品 | SHA-256 |
|---|---|
| V16 config | `ee51a774a79b02e2d55568e24b2f064050d56186654a9783c4ee9caaf2e4b700` |
| sparse scenario | `cc41099faee3a6773c6b3de407acc759c3f565d3fed1da840528807efaf78e36` |
| `run_manifest.json` | `e09faf0fe2f153757352319bc3c5449bdc73d9f748d20d8d2e6a0454ff56289a` |
| `live_trace.jsonl` | `bc22bf9db2eee32aedc46a2b903bbdfae19b25b7d662f890fd42d2c905076857` |
| `trajectory.csv` | `7cafae42a2a296520e86a1dbcfe0ad2161ad5b35375156dd4be31826996d3862` |
| `camera_video_manifest.json` | `8fe0a0d5e4340ddacd768873eda440f6aad12450c5b51cfd02d8359ea6f7d48e` |

服务器 Ruff 检查通过；功能相关的控制、Gazebo、实时显示、视频与 replay 测试为
`53 passed`，完整测试集为 `241 passed`。所有运行和测试均在服务器环境执行。

## 声明边界

- 真正参与控制的是 measured-pose lifted online robust distance CBF 及其逐步精确
  action projection；相机、dashboard 和视频只读，不向 controller 回传数据。
- `q_h=0.30 m` 是距离函数 `h` 的米制可视化 level set。它不是 q-SBC 中无量纲的
  概率界 `q`，不得把青色线称为已求解的 q-SBC 概率证书。
- 固定几何 q-SBC 仍为 `not_ready`；本片不声称形式不变性、概率安全、随机几何
  泛化、独立 replay、ROS2 或实车安全。
- 本片是 seed `7320014` 的单次 Gazebo ODE 经验结果，不取代 V13 的 32-pattern
  多 seed 与 replay 证据。
- 外层 V16 attempt1 只因 Gazebo 关闭边界留下一个未完成相机 writer buffer 而失败，
  其服务器目录保留为 `episode_seed_7320014_attempt1`；attempt2 是修复后的首个成功
  结果；当前正式交付 attempt3 使用最终源码并真实通过一次受控的启动前重试。
  三个目录均未覆盖或删除。
