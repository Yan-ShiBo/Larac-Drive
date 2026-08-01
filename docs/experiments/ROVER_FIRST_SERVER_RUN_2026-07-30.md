# Rover-first 正式服务器运行记录（2026-07-30）

## Material Passport

- Material type: immutable-run evidence summary
- Research contract: `docs/mathematics/RESEARCH_CONTRACT.md`
- Source archive SHA-256:
  `FDACFEEE873BC16C7E7101AB63725DDECDF0BB1C6AF05282E030C5C6C8AEB347`
- Source snapshot:
  `/data/users/csynth/cbf_rover_runs/source_20260729T185017Z_fdacfeee873b`
- Result root:
  `/data/users/csynth/cbf_rover_runs/results_20260729T185017Z_fdacfeee873b`
- Execution boundary: server only
- Final research status: `not_ready_at_PB1_PAC`
- Controller artifact: none
- q-SBC certificate artifact: none
- Gazebo or real-vehicle claim: none

## 结论

本轮完整执行了冻结合同中的服务器代码门禁、五个正式 SAC 训练、V1 唯一
checkpoint 选择，以及 PB1 的三个且仅有的 PA1/PS1 阶段。V1 选择 seed 11；
但是三个稀疏多项式阶段都无法在独立 20,000 点 selection 集上满足
`L_inf <= 0.05 m/s`，因此研究链按合同停止为 `not_ready`。

没有生成 PAC controller candidate，也就没有合法输入可供 CV1 冻结或 G5 q-SBC
综合。禁止把本轮结果描述为 controller 已验证、证书不可行或系统已安全；本轮只
证明既定 PB1 support schedule 未通过其模仿门禁。

## 服务器环境

- Host: `ai`
- OS: Ubuntu kernel `5.15.0-139-generic`
- CPU: 2 x Intel Xeon Gold 6148，80 logical CPUs
- GPU:
  - GPU 0: RTX 4090, 24,564 MiB
  - GPU 1: RTX 4090, 24,564 MiB
  - GPU 2: RTX 4090, 49,140 MiB
  - GPU 3: RTX 3090, 24,576 MiB
- NVIDIA driver: `570.133.07`
- Python: `3.13.14`
- NumPy: `2.4.4`
- SciPy: `1.18.0`
- PyTorch: `2.8.0+cu128`
- stable-baselines3: `2.9.0`
- Gymnasium: `1.3.0`
- MOSEK: `11.2.2`
- PyYAML: `6.0.3`

完整环境记录位于结果根目录的 `validation/environment.log`。最终 source
snapshot 已递归移除写权限；环境记录中的 `SOURCE_WRITABLE_FILES` 后无条目。

## 代码门禁

| Gate | Server result | Exit |
|---|---:|---:|
| `pytest tests/unit tests/math -m "not research_slow" -q` | 52 passed, 2 deselected | 0 |
| `pytest tests/unit tests/math -m research_slow -q` | 2 passed, 52 deselected | 0 |
| `ruff check src tests` | All checks passed | 0 |
| `python -m cbf_rover.entrypoints.research --help` | CLI loaded | 0 |

pytest 的唯一 warning 是只读 source snapshot 无法创建 `.pytest_cache`；这符合
快照冻结设计，不影响测试结果。日志与退出码保存在 `validation/`。

## ST1 正式训练

全部正式训练使用 600,000 steps、固定五个 training seed、
`CUBLAS_WORKSPACE_CONFIG=:4096:8`、确定性 PyTorch 设置和冻结源码。训练分配为：

| Seed | Device | Checkpoint SHA-256 |
|---:|---|---|
| 11 | `cuda:1` | `762cd3a4160da25cfba2d6241685b626f973dbaf8b1aade235188c43c2e6d199` |
| 23 | `cuda:2` | `b22b2bd93e4fedbe2fad7280ac15aa43e4f294e6a719816a399ef4d3da3bdc06` |
| 47 | `cuda:3` | `ec788b1f0401c9a4aca14efe8e8a2deca3246a62b622065f2c9428d3f78aea7b` |
| 89 | `cuda:1` | `8b03e58f83d8ec1f014de658c3981e2d03227fb17fccd963bbe8fd4464f717fb` |
| 131 | `cuda:3` | `b5a018ed4fc3e3d25d564a3991861686dbddcb64d9c6c725b2277e74dd96d977` |

正式 GPU 运行之前的 CPU device benchmark 在产生 checkpoint 前终止，并以
`production/sac/ABORTED_BEFORE_CHECKPOINT_DEVICE_BENCHMARK` 标记。20,000-step
GPU smoke 只用于设备吞吐检查，位于 `smoke/`，未进入 V1。

## V1 唯一选择

V1 在 CPU 上使用 evaluation seeds
`[1009, 2027, 3037, 4051, 5059]`，每个 evaluation seed 20 episodes，即每个
checkpoint 100 episodes。全部训练和 exact contract hash 在评估入口内重新校验。

| Training seed | Goal count | Goal rate | Median successful steps | Hard failures |
|---:|---:|---:|---:|---:|
| 11 | 100/100 | 1.00 | 340 | 0 |
| 23 | 0/100 | 0.00 | n/a | 0 |
| 47 | 0/100 | 0.00 | n/a | 0 |
| 89 | 0/100 | 0.00 | n/a | 0 |
| 131 | 0/100 | 0.00 | n/a | 0 |

选择结果为 seed 11。selection manifest SHA-256：
`f096912a6c72bb000b182adcba8f8b2c2cdf7942a55e9a64bfe0801e51ca242f`。
V1 是固定随机实验门禁，不是形式安全证明。

## PB1 / PA1 / PS1

三个阶段都使用冻结的 seed-11 checkpoint、`Uniform(Psi)` 的 50,000 个
support-search 样本、独立 20,000 个 support-selection 样本、beam width 32
和 HiGHS `L_inf` LP。每个阶段两轴各保留 32 个搜索候选。

| Stage | Catalog / support | Wall time | Result | Manifest SHA-256 |
|---:|---|---:|---|---|
| 1 | degree 3, 6 terms/axis | 20:08.67 | `not_ready` | `3a194e8c59765a28ef666cb114ee60d39e3c417950660ebf6ab7bfbf7f692299` |
| 2 | degree 3, up to 8 terms/axis | 13:24.01 | `not_ready` | `bf0561d64823fd37cc2b0ca327c59b5b1a469b9579e330a37193a5ad096b8141` |
| 3 | degree 4, 8 terms/axis | 2:34:43 | `not_ready` | `cbb646e19638409c751dad5b82a067ef9107933f9e687242c8cd2009ce2350de` |

三个 manifest 的统一失败字段均为：

```text
failure_reason = no_support_pair_passed_selection_gate
selection_hard_linf_mps = 0.05
```

失败发生在 support selection，早于 PAC refit；因此没有 `e_star`、PAC
candidate 或可冻结 controller。stage 3 是合同允许的最高次数，失败后没有自动
增加次数、项数、容差或重新抽样。

## 未运行阶段及声明边界

- CV1：未运行；原因是没有 PAC controller candidate。
- controller replay：未运行。
- G5 degree-4/6 q-SBC：未运行；原因是没有冻结 controller。
- certificate replay：未运行。
- Gazebo differential tracking：本轮未运行。
- 实车：不存在，也不在项目主线内。

因此本轮没有形式 q bound、没有 `epsbar`、没有 reach-avoid 概率声明，也不能把
“q-SBC 未运行”写成“q-SBC infeasible”。

## 固定证据位置

- `validation/environment.log`
- `validation/artifact_hashes.sha256`
- `validation/pytest_fast.log`
- `validation/pytest_research_slow.log`
- `validation/ruff.log`
- `production_gpu_deterministic/sac/seed_*/manifest.json`
- `production_gpu_deterministic/selection/selection.json`
- `production_gpu_deterministic/pac_stage_*/manifest.json`
- `production_gpu_deterministic/pac_logs/stage_*.log`

下一轮开始前应先分析“全局单一稀疏多项式近似 SAC”与当前参考策略几何之间的
失配，再明确修改新的研究合同。本轮在此固定并暂停。
