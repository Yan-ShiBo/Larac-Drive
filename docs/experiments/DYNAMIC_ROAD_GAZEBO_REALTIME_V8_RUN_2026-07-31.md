# Gazebo Classic 实时障碍证书 V8 优化与服务器验证

## Material Passport

- Origin Skill: `experiment-agent`
- Origin Mode: `run`
- Origin Date: `2026-07-31`
- Verification Status: `VERIFIED`
- Version Label: `gazebo_realtime_v8`

## 结论

V8 已把 Gazebo 物理闭环从 V7 的约 `0.25x` 提升到稳定的 `1x` 墙钟运行，
同时保留原有物理、controller、在线 CBF、动作投影、时间戳对齐和实时显示
合同。

两次独立服务器运行均：

- 到达道路终点并激活全部 5 个随机障碍；
- 0 碰撞、0 道路越界；
- 通过 20/20 单次物理与实时显示门禁；
- 保持 `dt=0.05 s`、每控制步 50 个 `0.001 s` ODE physics iterations；
- 不允许受证 controller 后置 clipping；
- 由只读 dashboard 实时显示测量姿态、活动障碍、膨胀证书边界和 CBF 余量。

Replay-v4 对两个独立结果通过 19/19 门禁，状态为 `accepted`。

## 1. 瓶颈诊断

V7 的世界文件已经设置：

```text
real_time_update_rate = 0
```

因此慢速不是人为实时限速。V8 首先加入默认关闭的
`CBF_ROVER_GAZEBO_BRIDGE_TIMING=1` 分阶段计时，并在相同 V7 world、相同
固定命令、20 个控制步下执行 bridge/physics-only 探针。

| 指标 | V7 同步路径 | V8 iteration 同步 | 改善 |
|---|---:|---:|---:|
| 平均每步墙钟 | 196.164 ms | 13.104 ms | 14.970× |
| 中位每步墙钟 | 202.168 ms | 11.303 ms | 17.886× |
| P95 | 202.556 ms | 16.658 ms | 12.160× |
| 探针实时因子 | 0.25489 | 3.81558 | 14.970× |
| 20 步物理推进 | 1000 iterations / 1.0 s | 1000 iterations / 1.0 s | 完全保留 |

基线中，velocity publish 与 world-control publish 都小于 `0.1 ms`；约
`0.20 s` 几乎全部位于同步等待。每个控制步只得到一条低频
`world_stats`，因此 V7 把统计发布周期放进了关键路径。

诊断证据：

```text
/data/users/csynth/cbf_rover_runs/diagnostics_gazebo_v8_baseline_415fbad8
/data/users/csynth/cbf_rover_runs/diagnostics_gazebo_v8_pose_iterations_7fdded77
```

这些探针不运行 controller 或 CBF，只能作为性能定位证据。

## 2. V8 同步边界

伴随状态插件现在在私有 `PosesStamped` 消息中同时返回：

```text
pose timestamp = World::SimTime()
pose id        = World::Iterations()
pose value     = measured WorldPose()
```

每个 `STEP` 的目标 iteration 由上一条已验收姿态加 50 得到。桥接器发布原
`[v, omega]` 与 `multi_step=50` 后，轮询状态插件，直到：

```text
pose_iterations >= target_iterations
```

旧响应或丢失响应只触发状态重请求，不会再次推进物理。`world_stats` 仍用于
启动健康检查，但不再是逐步完成门禁。公共 `GZCB1` 行协议没有变化。

未改变的研究参数包括：

```text
ODE solver iterations          40
physics step                   0.001 s
physics iterations / command   50
controller / CBF dt            0.05 s
maximum lookahead speed        0.5 m/s
post-projection clipping       forbidden
live display feedback          false
```

## 3. 服务器命令

正式运行使用同一冻结源码和同一 Release 构建：

```bash
PYTHONPATH=<source>/src python -m cbf_rover.entrypoints.research \
  simulate-gazebo-classic-road-cbf-live \
  --config <source>/configs/experiments/dynamic_road_gazebo_classic_v8.yaml \
  --bridge-binary <build>/cbf_rover_gazebo_classic_bridge \
  --output-dir <new-result-directory> \
  --no-gif
```

独立 replay 使用：

```bash
PYTHONPATH=<source>/src python -m cbf_rover.entrypoints.research \
  validate-gazebo-classic-road-replay \
  --config \
  <source>/configs/experiments/dynamic_road_gazebo_classic_replay_v4.yaml \
  --run-manifest <v8-a>/run_manifest.json \
  --run-manifest <v8-b>/run_manifest.json \
  --output-dir <new-replay-directory>
```

所有调用均有显式硬超时。没有自动重跑崩溃实验，也没有连接或停止其他用户的
默认 Gazebo。

## 4. 两次独立 V8 结果

| 指标 | V8-A | V8-B |
|---|---:|---:|
| status | `provisional_success` | `provisional_success` |
| termination | `goal_reached` | `goal_reached` |
| transitions | 830 | 831 |
| simulated duration | 41.50 s | 41.55 s |
| wall duration | 41.507922 s | 41.553600 s |
| real-time factor | 0.999809 | 0.999913 |
| live updates | 209 | 209 |
| single-run gates | 20/20 | 20/20 |

两次运行的合并最坏值：

| 物理 / 证书指标 | 合并最坏值 | 门限 |
|---|---:|---:|
| minimum lifted barrier | 0.302792940 m | ≥ -1e-9 |
| minimum CBF residual | -1.11e-16 m/s | ≥ -1e-9 |
| minimum measured contraction residual | 0.060261565 m | ≥ -1e-9 |
| maximum position tracking residual | 0.012985207 m | ≤ 0.04 |
| maximum heading tracking residual | 0.046313541 rad | ≤ 0.10 |
| collision / road departure | 0 / 0 | 0 / 0 |

相对 V7-B 的 `0.24988`，V8-B 的完整墙钟实时因子提高 `4.002×`。探针可以
快于 1x，但正式 dashboard 配置继续使用
`not_faster_than_one_times_simulated_time` pacing，所以用户看到的是稳定的
真实时间仿真，而不是加速回放。

## 5. 运行期间外部显示观察

V8-B 控制进程存活期间，独立 shell 每约 `0.2 s` 请求一次：

```text
http://127.0.0.1:11469/snapshot.json
```

得到 184 次成功响应和 176 个不同文件哈希。运行自身的 manifest 记录：

```text
status                 completed
expected updates       209
actual updates         209
terminal alignment     passed
observer only          passed
maximum snapshot       47,777 bytes
```

观察器只读取 HTTP，不向 controller、CBF 或动作路径写入数据。

## 6. Replay-v4

Replay-v4 的 19 个门禁全部通过：

```text
transition-count spread          1        <= 2
simulated-duration spread        0.05 s   <= 0.10 s
final center-position spread     0.000645461 m <= 0.05 m
final heading spread             2.666e-7 rad  <= 0.10 rad
manifest content hashes          valid
referenced plan/world/trace      valid
runtime contract identity        matched
```

两个 ODE trace 不要求逐字节一致；该结论是有界经验重复性，不是形式确定性。

## 7. 代码与集成门禁

服务器验证结果：

```text
Ruff                                      passed
fast unit + mathematics suite             105 passed
research_slow MOSEK/SOS suite              3 passed
focused Gazebo/live/replay tests           12 passed
Gazebo bridge integration tests             2 passed
V8 single-run physical + display        20/20 passed twice
V8 bounded replay                       19/19 passed
```

快速套件的首次命令因 SSH 吞掉 pytest marker 表达式引号而得到
`0 tests ran`；该日志被保留。随后使用等价、无引号歧义的文件边界运行：
快速套件忽略唯一的 `test_sos_build.py`，慢套件只运行该文件。没有失败测试
被静默重跑。

## 8. 身份与结果位置

运行源码快照：

```text
/data/users/csynth/cbf_rover_runs/source_gazebo_live_v8_run_candidate_dff9da9d
SHA-256 dff9da9d20810acbe42dfa9b04c2d3b7e4f50a0c194773fbdd2919b4fe9f731b
```

Release bridge 与状态插件：

```text
bridge e7e3eef7ba202fb123051ad03a897a5a8a92f535862f2e25bc0f6e781de7f422
plugin 888a8e4b746adec1090886128c7a98a23fd574f3174e4c6e10d9e4805efb5f1b
```

结果目录：

```text
/data/users/csynth/cbf_rover_runs/results_gazebo_live_v8_candidate_a
/data/users/csynth/cbf_rover_runs/results_gazebo_live_v8_final_b
/data/users/csynth/cbf_rover_runs/results_gazebo_live_v8_replay_v4
/data/users/csynth/cbf_rover_runs/results_gazebo_live_v8_render_b
```

关键哈希：

| artifact | SHA-256 |
|---|---|
| V8 run config | `6db4e3789639a056a7dfda629d61f181bb8fc5f1aee8d52864c4debba045a112` |
| replay-v4 config | `5772e5882c9866b7eef928fd1fc2e73a87e99127c2346c6bea5588fce05ff199` |
| shared SDF world | `a8fae1a13ffedec58d7176c9753b4817c8bc27292cb168dd35fc9fca2d801eb0` |
| V8-A run manifest file | `49614f8fe5a9256c84b018739d6bb2b52a2e378459643f5105cf46341b78fa9b` |
| V8-A trace | `21aecd7f16ea6e05b6bc95b17c13b81763a5196bab0625a831a8afbcb3c5ef40` |
| V8-B run manifest file | `bdd503b4bb3bbb22d42752b5c204e41c2ad5d19aa85dee2609a6d1ff2778ef2b` |
| V8-B trace | `2d277cae0c5a4440ee83625f772ad7225362568b6e2e3148044e9e1dbd01ca9f` |
| replay-v4 report file | `4110c139cf2df8db94e7ad77e8ca26f28e3443db6185accf123c06eeaeb995c1` |
| replay-v4 report content | `6d42659a7e8acf31628a16b975ef4b6bc8835344735aeafa48dfc4e8002b0afc` |
| verified 1× GIF | `3666c5f3e4e52a7c9cbbb7f395879892347b2db5d5df1a235190a6788799559a` |
| verified final PNG | `03f4def7f9edf0bbba58a958abb8260865b73375476a5796edbb3d3d26dbac07` |
| render manifest | `16dd521930b3b43d89248363b97d88999eb8ae94ce236c485c4c3c96b2d9c5f1` |

## 9. 声明边界

V8 验证的是：

- Gazebo Classic 11 ODE 的测量状态差速物理闭环；
- 在线距离 CBF 在每个时间戳/iteration 对齐姿态上的运行时评价与动作投影；
- 实时、只读障碍证书显示；
- 两次同合同运行之间的有界经验重复性。

V8 不验证：

- 固定几何 q-SBC 概率界；
- 随机几何 q-SBC；
- ROS2 集成；
- 实车安全；
- 形式化差速跟踪误差界。

固定几何 q-SBC 仍保持
`controller_validated_qsbc_not_ready_stage1_unknown`，不能由本次经验仿真
成功替代。
