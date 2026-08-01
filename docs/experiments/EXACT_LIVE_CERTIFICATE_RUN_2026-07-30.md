# M1 exact 障碍证书实时仿真记录（2026-07-30）

## Material Passport

| 材料 | 身份 / SHA-256 |
|---|---|
| 只读源码快照 | `/data/users/csynth/cbf_rover_runs/source_20260730T102302Z_7541f4748c33` |
| 源码归档 | `7541f4748c334be9eb97dcf4e66022c0e93d5a6e3686846152c11be74f75cdaa` |
| 只读结果 | `/data/users/csynth/cbf_rover_runs/results_20260730T102302Z_7541f4748c33` |
| C2 controller 文件 | `df0dbfd2cc5b3aa0a049b5dbb6e7d0360c6b260801e183e95acfb7a9bccf715d` |
| C2 controller 内容 | `b0cacedfcd4b884961a786b24e84b2f0f33bbc4ac81aef0b1ff1b0dcfe285984` |
| G7 W14 certificate 文件 | `2fe58a62815ea0f0835f7916b8b861d3b1762a57d17e8780f887568e45b49849` |
| G7 W14 certificate 内容 | `b01a3a715d480931f4fe4a1a3a8ab512eb03beb012be210ffc5dcb096ff25e0d` |
| 独立验证报告 | `63d24956f11a465bdbd15f6ba3973330861c577d2b17279bac4a1b820474a404` |
| 实时 trace | `8e504e8a38f292ef9e60005866ed4e93f8b0601fd435f17a796a9f037b9491ef` |
| 1× GIF | `ec8993e7e9b60951a5f64b56b006b8f1b5764fc3d8b1cf2c7aa3f24560be87b0` |
| 终帧 PNG | `e155a4d587fc95cf3986812404162b20e8432ab0d197a9c32a31c8299c5586be` |
| run manifest 文件 | `921791bf58334bdf2832783f8410f7b1d5716b535028fb2724c18a561670ca82` |
| run manifest 内容 | `a142cb7ec03d3d6dd98ab5ab57719483da2782991744e446b30a3c9b3321d489` |

本轮所有 Python、pytest、求解、仿真和渲染均在 `ai-server` 运行。本机只做
源码编辑、静态 diff 检查、归档、传输和结果查看。每个正式服务器调用都有显式
硬超时；长任务按约 30 秒间隔观测 PID、CPU、RSS、产物和退出文件，没有静默
重试。

## 1. 证书状态

G7 W14 是独立验证的 G4 无限时域 q-SBC：

```text
q = 0.8765194159459675
epsbar = 1e-6
P(hit Xu before Xg) <= q
P(hit Xg before Xu) >= 1-q = 0.1234805840540325
E[tau] <= q/epsbar = 876519.4159459675 steps
```

它的状态是 `formal_valid_not_ready`，原因是 `q>0.10`。这不影响“存在正式
q-SBC”和 exact monitor-only 运行，但不能写成首片达到目标界。`q` 不是当前
状态的瞬时碰撞概率，`W(x)<1` 也不是单点安全分类器。

后续 G8 标量乘子路线得到 `q=0.8647346899465488`，但独立复核在
`drift_lower_transit_D_1_2` 返回 `unknown`，所以 G8 候选不是证书。密网格
G9 诊断进一步表明，DX2 的低 q 是粗网格高系数过拟合：

| 诊断 | 结果 |
|---|---:|
| W14 monomial，201×161，margin `1e-4` | `q=0.4948269503` |
| W16 monomial，201×161，margin `1e-5` | `q=0.3227016942` |
| W14 Chebyshev，201×161，margin `1e-5` | `q=0.2145613783` |
| W16 Chebyshev | 520 秒 time limit，无候选 |

这些诊断均为 `formal_claim_allowed=false`，没有被装成证书。

## 2. 服务器门禁

源码快照先通过：

```text
ruff check --no-cache .                         all checks passed
pytest -m "not research_slow and not ros2"      75 passed
pytest -m "research_slow"                       3 passed
```

服务端环境为 Ubuntu 20.04.6、Python 3.13.14、NumPy 2.4.4、SciPy 1.18.0、
Matplotlib 3.11.0、Pillow 12.2.0。

## 3. 实时 exact 运行

正式命令：

```bash
timeout --signal=TERM --kill-after=120s 1800s \
env PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH=/data/users/csynth/cbf_rover_runs/source_20260730T102302Z_7541f4748c33/src \
/data/users/csynth/cbf_rover_runs/results_20260729T184104Z_4525410e0fbc/environment/venv/bin/python \
-m cbf_rover.entrypoints.research simulate-exact-certificate-live \
--controller /data/users/csynth/cbf_rover_runs/results_20260730T102302Z_7541f4748c33/inputs/frozen_controller.json \
--certificate /data/users/csynth/cbf_rover_runs/results_20260730T102302Z_7541f4748c33/inputs/validated_certificate.json \
--output-dir /data/users/csynth/cbf_rover_runs/results_20260730T102302Z_7541f4748c33/exact_live_1x \
--seed 1009 --display-stride 4
```

每 `0.05 s` 输出一条 JSON 事实记录，包含实际 state、nominal action、N1
noise、`W(x)`、q、epsbar、certificate/controller 哈希和 monitor-only 标志。
certificate evaluator 从未修改 action，也没有 post-controller clipping。

结果：

```text
status                         goal_reached
steps                          1366
simulated duration             68.30 s
wall elapsed                   68.300064228 s
frames                         1367
minimum obstacle-set distance  0.2043255151 m
W minimum / maximum            0.2137211890 / 0.8484764999
W final                        0.2141114145
```

无等待 smoke run 与 1× wall-clock run 的 trace SHA-256 完全相同，step count
和 final W 也相同，证明实时节拍没有改变随机轨迹。

本机只读副本位于：

```text
E:\6_Engineering_Projects\CBF-Rover\runs\20260730T102302Z_exact_live_1x
```

## 4. ROS2 / Gazebo 边界

服务器有 Gazebo Classic 11.15.1 和 ROS1 Noetic，但没有 `/opt/ros/humble`、
`ros2`、`colcon` 或 `rviz2`。因此本轮没有伪称 ROS2 build、Gazebo 闭环或
RViz 通过。仓库已实现 ScenarioSpec 注入、证书事实 topic 和 RViz marker，
但它们仍等待具备 ROS2 Humble 的服务器环境验证。

这不影响本轮 M1 exact 仿真：M1 是用户确认的首片数学系统，Gazebo 差速跟踪
本来就是后续经验映射，且不能继承 exact q-SBC 概率界。
