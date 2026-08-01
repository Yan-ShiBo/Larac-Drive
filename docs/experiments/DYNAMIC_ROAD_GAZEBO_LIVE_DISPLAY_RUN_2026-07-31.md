# 2026-07-31 Gazebo 动态公路障碍证书运行时显示

## Material Passport

- Origin Skill: experiment-agent
- Origin Mode: run
- Origin Date: 2026-07-31
- Verification Status: VERIFIED
- Version Label: gazebo_live_display_v7

## 1. 结论

V7 已在服务器上的 Gazebo Classic 11 ODE 物理场景中完成目标闭环：

- 小车沿双车道公路运行并到达终点；
- 5 个由固定 seed 生成的随机障碍物按 controller activation 边界依次出现；
- 障碍物激活后，在线 lifted distance CBF 每个 `0.05 s` 控制步都从时间戳
  对齐的 Gazebo 实测位姿重新计算；
- 运行中的 loopback HTTP 页面持续显示小车、轨迹、物理障碍物、膨胀证书边界、
  最小 barrier、收缩余量、`[v, omega]` 与 CBF intervention；
- 显示端位于 action filter 之后，只读证书帧，不修改 action，也不向 controller
  回传数据；
- 最终运行通过 20/20 单次物理与显示门禁，两个隔离 V7 运行通过 replay-v3
  的 19/19 有界重复性门禁。

这是经验 Gazebo 闭环结果。它不是 q-SBC 概率证明，不是 ROS2 结果，也不是
实车安全证明。

## 2. 运行时数据流

```text
measured Gazebo pose + world timestamp
                  |
                  v
       lifted online distance CBF
                  |
                  v
  projected lookahead velocity -> [v, omega]
                  |
                  v
       paused fixed-step Gazebo command
                  |
                  +--> immutable frame observer
                              |
                              v
                  atomic snapshot.json + events.jsonl
                              |
                              v
                   loopback Canvas dashboard
```

显示发布器没有从右侧返回 controller 的边。显示写出发生在控制帧完成之后；
其耗时只增加墙钟时间，不改变 Gazebo 的固定物理步长、投影结果或证书值。

## 3. 活动配置与运行命令

- 运行合同：`configs/experiments/dynamic_road_gazebo_classic_v7.yaml`
- replay 合同：
  `configs/experiments/dynamic_road_gazebo_classic_replay_v3.yaml`
- Gazebo master：`http://127.0.0.1:11468`
- live dashboard：`http://127.0.0.1:11467/`
- seed：`7321609`
- simulated display stride：4 steps，即 0.20 s
- dashboard refresh：0.20 s

V7 配置自动启用 live display 与 wall-clock pacing；命令不需要重复传显示参数：

```bash
PYTHONPATH=src python -m cbf_rover.entrypoints.research \
  simulate-gazebo-classic-road-cbf-live \
  --config configs/experiments/dynamic_road_gazebo_classic_v7.yaml \
  --bridge-binary \
  build/gazebo_classic_bridge/cbf_rover_gazebo_classic_bridge \
  --output-dir /absolute/new/result/directory \
  --no-gif
```

远程查看时，在本机建立 SSH tunnel：

```bash
ssh -L 11467:127.0.0.1:11467 ai-server
```

随后在本机打开 `http://127.0.0.1:11467/`。服务在 terminal snapshot 之后停止，
但 `index.html`、`snapshot.json` 和 `display_events.jsonl` 会保留在结果目录。

## 4. 最终 V7-B 结果

| 指标 | 结果 |
|---|---:|
| status | `provisional_success` |
| termination | `goal_reached` |
| acceptance gates | 20/20 |
| transitions / frames | 831 / 832 |
| simulated duration | 41.55 s |
| wall duration | 166.2793 s |
| achieved real-time factor | 0.24988 |
| activated obstacles | 5/5 |
| collisions / road departures | 0 / 0 |
| filter intervention steps | 183 |
| minimum lifted barrier | 0.3027826492 m |
| minimum CBF residual | `-1.11e-16 m/s` |
| minimum measured contraction residual | 0.0602584155 m |
| maximum position tracking residual | 0.0129852071 m |
| maximum heading tracking residual | 0.0452294707 rad |
| maximum roll / pitch | `3.57e-5 / 0.00636015 rad` |
| post-projection command clipping | none |

“运行时显示”表示仿真进程存活期间页面持续获得新证书帧，不等于 Gazebo 以
真实时间 1× 运行。服务器上的 ODE 固定步进约为真实时间的四分之一。

## 5. Live display 证据

| 显示事实 | 结果 |
|---|---:|
| status | `completed` |
| expected / actual updates | 209 / 209 |
| first / final sequence | 2 / 210 |
| first / final step | 0 / 831 |
| final simulated time | 41.55 s |
| captured activation ids | `obstacle_01` … `obstacle_05` |
| event sequence order | strictly increasing |
| event step order | strictly increasing |
| observer-only flag | true for every event |
| maximum atomic snapshot size | 47,834 bytes |

候选 A 的独立高频观察器在主仿真进程仍存活时读取了 1,565 个 HTTP 快照样本，
覆盖 starting、running 与 completed。它观察到 210 个不同序号，并在 terminal
grace 内 4 次读取到 `completed`；主 Python 进程最大 RSS 为 54,552 KiB。

关键文件 SHA-256：

```text
viewer index.html
5b247e4fcd77d1b22e9ee8294c06b190223a0dc58ce0b410ad9ad5295cedd5bd

display events
1e008f9d7439785738676494b366ff38cef8ab87cf02ec5bcbaf1ae91bcbee62

terminal snapshot
6c0c4c23fa48b354c5c112c35e350fe745d77928e45099d32aa06828c46a681e

live display manifest
23d3854bcf131fac6fe8afc438ab7ff12070f16c8687de4cefcf706727040dba
```

## 6. Replay-v3

候选 A 与最终 B 使用相同 V7 配置、base config、seed、obstacle plan、SDF world
和 runtime contract。独立 validator 重算两个 manifest content hash 和所有
plan/world/trace 文件哈希。

| replay 指标 | 结果 | 门限 |
|---|---:|---:|
| gates | 19/19 | 19/19 |
| transition-count spread | 1 | <= 2 |
| simulated-duration spread | 0.05 s | <= 0.10 s |
| final center-position spread | 0.00298597 m | <= 0.05 m |
| final heading spread | `5.80e-7 rad` | <= 0.10 rad |
| byte-identical trace | false | not required |

Replay report content SHA-256：

```text
40ecc454828c20793f80cd0de01166b8fe9ed4ca0da08f4c61cd55d641a81faa
```

## 7. 最终服务器门禁

```text
Ruff                                         passed
focused Gazebo/live-display/replay tests     11 passed
fast unit + mathematics suite                104 passed, 3 deselected
research_slow mathematics suite              3 passed, 104 deselected
Gazebo bridge cold starts                    5/5 passed
single-run physical + display acceptance     20/20 passed
bounded independent replay                   19/19 passed
```

## 8. 结果位置与身份

```text
server diagnostic V7-A:
/data/users/csynth/cbf_rover_runs/results_gazebo_live_v7_candidate_a

server final V7-B:
/data/users/csynth/cbf_rover_runs/results_gazebo_live_v7_final_b

server replay-v3:
/data/users/csynth/cbf_rover_runs/results_gazebo_live_v7_replay_v3
```

```text
V7 config SHA-256
e3cb5279bea3a0deb5821e7031c640d517072fed252a5a28ed76cef3a1009853

obstacle-plan content SHA-256
69a84355c38d50e490a2a342a17c1f1d293307351fae2ca049d36aebbd0afe4f

world SDF SHA-256
a8fae1a13ffedec58d7176c9753b4817c8bc27292cb168dd35fc9fca2d801eb0

bridge / state-plugin SHA-256
8b2cd458da327715ea88998448d4c87d31186ea4ddc5ca2d30b7b77681116b15
beda8448525cd13173454d0abda3740a490c9f6c73daf2f169ede16510ad24a0

V7-A / V7-B run-manifest file SHA-256
f8e7cfb95d672313f6897fce083f4570e8a314a3618d23d5b5c548e24e498912
2b76a9f1df567fc2715540ce2f0b61219f14b768a08fb0fd9873bf8eec73f79b
```

## 9. 声明边界

V7 验证了“Gazebo 物理闭环运行时可以同步显示障碍证书”这一经验工程目标。
它没有证明：

- tracking envelope 是形式不变集；
- online distance CBF 是 q-SBC 概率证书；
- Gazebo 接触动力学等价于真实车辆；
- ROS2/RViz 集成已经构建；
- 实车已经测试。

固定几何 q-SBC 链仍保持 `not_ready`，不能由 V7 的经验成功替代。
