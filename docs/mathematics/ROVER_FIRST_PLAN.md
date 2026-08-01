# Rover-first q-SBC 实施与服务器运行计划

## Material Passport

- Material type: executable research plan
- Scope: exact 2D Rover projection, SAC reference, PAC polynomial controller,
  global controller validation, infinite-horizon q-SBC
- Contract: `docs/mathematics/RESEARCH_CONTRACT.md`
- Execution boundary: server only
- Historical results imported: none
- Real-vehicle evidence: none

## 当前执行结论（2026-07-30）

最终只读源码快照已在服务器通过 52 个快速测试、2 个 MOSEK/SOS 慢测试和
Ruff。五个 600,000-step SAC 正式训练全部完成，V1 选择 seed 11（100/100
到达目标、0 个硬失败）。PB1 stage 1、2、3 均因
`no_support_pair_passed_selection_gate` 返回 `not_ready`，所以本轮依照第 2 节
停止条件终止；没有运行 CV1 或 q-SBC。完整证据与哈希见
[`ROVER_FIRST_SERVER_RUN_2026-07-30.md`](../experiments/ROVER_FIRST_SERVER_RUN_2026-07-30.md)。

## 1. 架构

```text
domain/
  planar.py                         M1/S1/P1/D1/N1 pure contracts

methods/polynomial_sbc/
  polynomial.py                     canonical polynomial/controller artifacts
  algebra.py                        exact polynomial and N1-moment algebra
  pac.py                            PB1/PA1/PS1
  bernstein.py                      exact-float global branch-and-bound
  controller_validation.py          CV1
  certificate.py                    G4 artifact/evaluator
  sos.py                            G5 convex SDP synthesis
  certificate_validation.py         solver-independent replay

adapters/
  exact/environment.py              exact simulator and Gym wrapper
  training/sac_reference.py         ST1/V1

entrypoints/
  research.py                       server research CLI
```

ROS2/Gazebo 只能依赖冻结的 domain/artifact 接口，不允许反向进入纯数学层。
learned RGB-D 保留在 `methods/baselines`，不进入首片依赖图。

## 2. 阶段与停止条件

1. 训练 5 个 SAC reference checkpoint；
2. 用 V1 唯一选择一个 checkpoint；
3. 运行 PB1 stage 1 的 PA1/PS1；
4. 对候选运行 CV1；
5. CV1 失败才进入下一 PB1 stage；
6. 首个 CV1 通过的 controller 立即冻结；
7. 对冻结 controller 运行 G5 `W degree 4 -> 6`；
8. 每个 solver 候选都运行独立 G5 replay；
9. 首个 `q<=0.10` 且 replay validated 的 artifact 为 `ready`；
10. 固定 source/result hashes 后停止。

以下状态立即停止当前链并报告 `not_ready`：

- 五个 checkpoint 均未通过 V1；
- 三个 PB1 stage 均未通过 PAC/CV1；
- 两个 W degree 均未达到首片 ready gate；
- 任何 hash、state order、unit、frame 或 config binding 不匹配；
- solver 或 validator 返回 unknown。

禁止自动混样、回看 q-SBC 挑 support、修改冻结 controller、增加既定次数或项数、
增加 W degree，以及重试同一 stochastic run 后挑最好结果。

## 3. 服务器目录

每次运行创建：

```text
/data/users/csynth/cbf_rover_runs/
  source_<UTC timestamp>_<git-or-tree-hash>/
  results_<UTC timestamp>_<source-hash>/
```

source snapshot 在首个命令运行后只读。result 下按：

```text
environment/
tests/
sac/
selection/
pac_stage_1/
controller_stage_1/
pac_stage_2/
controller_stage_2/
pac_stage_3/
controller_stage_3/
qsbc/
fixed_output/
```

保存命令、stdout/stderr、exit code、start/end time、hostname、GPU、Python 包、
MOSEK version、source hash 和输入 artifact hash。

## 4. 服务器门禁

先运行快速门禁：

```text
python -m pytest tests/unit tests/math -m "not research_slow" -q
python -m ruff check src tests
python -m cbf_rover.entrypoints.research --help
```

然后运行小预算 smoke，只检查 wiring，不发布研究结论：

```text
train-sac --timesteps 2000
fit-pac --search-samples 500 --selection-samples 200
```

smoke artifact 必须留在 `smoke` 目录并记录非生产参数，禁止复制到 fixed output。

生产运行严格使用配置默认预算。长任务没有自动 retry；hard timeout 仅终止并记录，
不把 timeout 改写成 infeasible。

## 5. 产物验收

controller fixed output 至少包含：

- `frozen_controller.json` 与 SHA-256；
- upstream checkpoint SHA-256；
- PB1/PA1/PS1 manifest 与数据 hashes；
- CV1 每个约束的 Bernstein tree；
- 1000-episode exact diagnostic；
- independent replay report。

certificate fixed output 至少包含：

- `certificate.json` 与 G4 theorem ID；
- controller content hash；
- `W,q,epsbar`；
- MOSEK version、tasks、Gram artifact 和 solver statuses；
- independent Bernstein replay trees；
- formal-valid 与 first-slice-ready 两级状态。

## 6. 当前边界

代码实现可以在本机编辑，但任何“通过”“可运行”“求解成功”只能引用服务器输出。
服务器未跑之前，状态只能是 `implemented_unvalidated`。
