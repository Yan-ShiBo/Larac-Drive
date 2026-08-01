# C2 controller 与 q-SBC 正式服务器运行记录（2026-07-30）

## Material Passport

- Material type: immutable-run evidence summary
- Research contracts:
  - `docs/mathematics/RESEARCH_CONTRACT.md`
  - `docs/mathematics/C2_PIECEWISE_CONTROLLER_CONTRACT.md`
- Git context: branch `codex/refactor-math-first-architecture`, base HEAD
  `40b384a883e01f7b370886c4f3baaedbc2cb1fa3`
- Source archive SHA-256:
  `252613ab1427e02f716166ebb696be08e6d6592013f0f177cc9b3cdbc76b15b8`
- Source archive entries: 184
- Source snapshot:
  `/data/users/csynth/cbf_rover_runs/source_20260730T083111Z_252613ab1427`
- Result root:
  `/data/users/csynth/cbf_rover_runs/results_20260730T083111Z_252613ab1427`
- Upstream DX1 result:
  `/data/users/csynth/cbf_rover_runs/results_20260730T080131Z_2bc7859fef91/diagnostics/dx1/diagnostic_report.json`
- Execution boundary: all Python, tests, controller validation, rollout and SDP
  execution occurred on server `ai`
- Controller status: `validated`
- q-SBC schedule status: `not_ready`
- Final research status:
  `controller_validated_qsbc_not_ready_stage1_unknown`
- q-SBC certificate artifact: none
- Gazebo or real-vehicle claim: none

## 结论

本轮成功完成了固定 C2 三分段低路由 controller 的构造、C2V1 形式验证、1000
条 exact 诊断 rollout 和一次独立 controller replay。冻结 controller 的 15 个
动作圆盘/robust D1 约束全部为 `validated`；1000/1000 条轨迹到达目标且 hard
failure 为 0。

随后对同一个只读 controller 完整执行 G5 的有限 `W degree 4 -> 6` schedule。
两个次数都在第一目标 `minimize_q` 返回 `SolutionStatus.Unknown`，因此按
fail-closed 规则记为 `infeasible_or_unknown`。没有进入第二目标，没有 numeric
candidate，没有独立 certificate replay，也没有 q-SBC certificate。

这不是“已证明不存在 q-SBC”，也不是“系统已安全”。它只说明：

1. C2 已经获得动作圆盘和 robust D1 的形式验证；
2. C2 在固定 1000 条 exact 诊断中表现成功；
3. 当前 G5 的 W4/W6 Putinar/MOSEK 实例没有给出可接受的最优解。

## 服务器环境与内存解释

- Host: `ai`
- OS: Linux kernel `5.15.0-139-generic`, x86-64
- CPU: 2 x Intel Xeon Gold 6148, 80 logical CPUs
- Python: `3.13.14`
- pytest: `9.1.1`
- NumPy: `2.4.4`
- SciPy: `1.18.0`
- PyTorch: `2.8.0+cu128`
- stable-baselines3: `2.9.0`
- MOSEK: `11.2.2`
- Ruff: `0.12.9`
- Formal SDP threads: 8

运行期间服务器总内存为 125 GiB，`available` 为 113--116 GiB。Linux 的
`free` 列曾显示约 1--7 GiB，但约 109--112 GiB 位于可回收
`buff/cache`；因此“free 较低”不表示服务器只剩不到 1 GiB 可用内存。

依赖解释器来自上一轮已记录的环境：

```text
/data/users/csynth/cbf_rover_runs/results_20260729T184104Z_4525410e0fbc/environment/venv/bin/python
```

每条命令都用 `PYTHONPATH` 显式绑定本轮只读 source snapshot，未从旧源码导入
CBF-Rover 包。

## 上游 DX1 绑定

构造入口重新验证了 DX1 内容哈希、正式文件哈希、decision ID、V1 training
seed 11、dense degree 1--6 全部失败和 `X0` rollout 全部走下方通道：

| Identity | SHA-256 |
|---|---|
| DX1 file | `ba2437602218e716a51311f4bddf45c0acb350e3de5cb25ed6f1f3201ca46483` |
| DX1 canonical content | `b1e77aab2d6d795bbbaddd61fbdef8e1f4a3ea123bfac54a0f6f727c874dcce2` |
| C2 construction source | `54328380a69e0b93b55785c8ab712a197d8fa96255e1c7d8dc2e704d3d0f11e1` |

DX1 只决定停止继续扩大全局 PAC，并提供“使用下方通道”的诊断依据；它不是
C2V1 或 q-SBC 证明。

## 代码门禁

| Gate | Server result | Exit |
|---|---:|---:|
| Ruff, no cache | All checks passed | 0 |
| pytest excluding `test_sos_build.py` | 62 passed | 0 |
| MOSEK/SOS slow test file | 3 passed in 2.99 s | 0 |

慢测试首次执行即通过；没有测试重跑。source snapshot 已递归移除写权限，并以
`PYTHONDONTWRITEBYTECODE=1`、pytest `-p no:cacheprovider` 避免写入。

## C2 candidate 身份

| Field | Value |
|---|---|
| Schema / kind | `2 / frozen_piecewise_polynomial_controller` |
| Controller ID | `CBF_ROVER_C2_LOWER_ROUTE_V1` |
| Controller content SHA-256 | `b0cacedfcd4b884961a786b24e84b2f0f33bbc4ac81aef0b1ff1b0dcfe285984` |
| Controller file SHA-256 | `df0dbfd2cc5b3aa0a049b5dbb6e7d0360c6b260801e183e95acfb7a9bccf715d` |
| Build manifest file SHA-256 | `d4377f29cac1daa305f98e07a9ba6240e25402e11d751bc1b1e97503aca49541` |
| Build manifest content SHA-256 | `76fed8bf4b1ef828fd0fdaba5890fb6e7dd081fa9d63d05433bb409b37ed9892` |

三个闭区间为 `[-1,0.8]`、`[0.8,3.2]`、`[3.2,4]`。运行时在共享边界按
artifact 顺序选择首个 piece，形式验证则覆盖相邻两个闭包。certificate
feedback 未参与构造。

## C2V1 形式与经验门禁

Validation report：

- file SHA-256:
  `754eeae6db1ec375cbfb23aa363fe59c8f574dae64cd4e81c87b9b893c251566`
- canonical content SHA-256:
  `6ccc6bf6252e085bffd9df253fa37d9e89b2958ec97d4c04b7d1250224b93ebb`
- decision:
  `C2V1_PIECEWISE_BERNSTEIN_AND_1000_ROLLOUTS`

| Formal gate | Count / minimum visited lower bound | Status |
|---|---:|---|
| Action disk, one per piece | 3; minimum `0.00496666666666662` | validated |
| Robust D1, four per piece | 12; minimum `0.0002500000000003055 m` | validated |
| Total | 15 | validated |

全部约束在根盒的 Bernstein lower bound 已超过要求，因此各自只需一个 accepted
leaf；没有 `unknown`。

| Exact diagnostic | Result |
|---|---:|
| Episodes | 1000 |
| Goal reached | 1000 |
| Goal rate | 1.0 |
| Hard failures | 0 |
| Steps, min / median / max | 1198 / 1334.5 / 1561 |

独立 controller replay 不读取原 leaf tree，也不重复经验 rollout：

- status: `validated`
- report file SHA-256:
  `12d6be32e2b69b5cc2f8ee6f2815ff166eeb33cb24e19c196552e59063b0ceb1`
- report canonical content SHA-256:
  `fbd719432cf02895da8f297b9250440172aad390c02ef94212d1cd681edaa4ad`

冻结文件与 candidate 的文件 SHA-256 相同，内容身份未漂移。

## G5 q-SBC 有限 schedule

固定输入 controller content SHA-256 为
`b0cacedfcd4b884961a786b24e84b2f0f33bbc4ac81aef0b1ff1b0dcfe285984`。
每个 objective 上限 600 秒，8 线程，solver relative gap `1e-8`，
`epsbar in [1e-6,1]`，W coefficient bound `[-1000,1000]`。

| W degree | Equality constraints | PSD matrices / scalarized | Stage 1 | Attempt status |
|---:|---:|---:|---|---|
| 4 | 1530 | 162 / 8982 | `SolutionStatus.Unknown` | `infeasible_or_unknown` |
| 6 | 3066 | 162 / 34260 | `SolutionStatus.Unknown` | `infeasible_or_unknown` |

MOSEK 内点阶段分别运行约 1.63 s 和 4.65 s；主要墙钟时间用于构造并写出
分段 SOS 模型。两个 attempt 都只产生 `stage1_min_q.ptf` 与 manifest：

| Artifact | File SHA-256 |
|---|---|
| W4 manifest | `bb11fba832c38a67f89a05b95a2ba0e55cf4c9229f4c1c15c4dc6d8ebc5b6247` |
| W6 manifest | `21e8fc6cd397806a2fd10bd14225880047147f9e47512cea5b724a069c4578d8` |
| Schedule manifest | `ac2502d438daf6d24ca2eda6d95943d8bef793b60c6acdfa1c4fb92109e1fb35` |
| Solver log | `cb62c8527d24967ee3d320673cc66ef546998de73c7dc7ad2c40788d812f4b4d` |

Schedule canonical content SHA-256 为
`93da6aa8eb749999e2c3b7e582dc79b01736b57969ca3aa195843363ebbd0a17`。
正式命令退出码为 2，表示合同定义的 `not_ready`，不是进程崩溃。

solver 日志中的迭代目标值不是合法的 `q`：最终 solution status 为
`Unknown`，manifest 因而没有记录 `q` 或 `epsbar`。两个目录均确认不存在
`numeric_candidate.json` 和 `certificate.json`。

## 正式命令

以下命令均在 server `ai` 执行，且 source 路径固定为本轮只读快照。

```bash
timeout --signal=TERM --kill-after=10s 300s \
  .../venv/bin/ruff check . --no-cache

timeout --signal=TERM --kill-after=10s 300s \
  env PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=<source>/src \
  .../venv/bin/python -m pytest -q -p no:cacheprovider \
  --ignore=tests/math/test_sos_build.py

timeout --signal=TERM --kill-after=30s 900s \
  env PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=<source>/src \
  .../venv/bin/python -m pytest -q -p no:cacheprovider \
  tests/math/test_sos_build.py

timeout --signal=TERM --kill-after=10s 300s \
  env PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=<source>/src \
  .../venv/bin/python -m cbf_rover.entrypoints.research \
  build-c2-controller --diagnostic-report <DX1-report> \
  --output-dir <result>/controllers/c2_candidate

timeout --signal=TERM --kill-after=30s 1800s \
  env PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=<source>/src \
  .../venv/bin/python -m cbf_rover.entrypoints.research \
  validate-controller --candidate <result>/controllers/c2_candidate/controller_candidate.json \
  --output-dir <result>/controllers/c2_validation

timeout --signal=TERM --kill-after=10s 600s \
  env PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=<source>/src \
  .../venv/bin/python -m cbf_rover.entrypoints.research \
  replay-controller --validation-report <result>/controllers/c2_validation/validation_report.json \
  --controller <result>/controllers/c2_validation/frozen_controller.json \
  --output-dir <result>/controllers/c2_replay

timeout --signal=TERM --kill-after=60s 3000s \
  env PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=<source>/src \
  .../venv/bin/python -m cbf_rover.entrypoints.research \
  solve-qsbc --controller <result>/controllers/c2_validation/frozen_controller.json \
  --output-dir <result>/certificates/g5_schedule \
  --time-limit-s 600 --threads 8
```

第一次 q-SBC 后台启动在打开 `<result>/certificates/g5_schedule.log` 时发现父
目录不存在，shell 在执行 Python/MOSEK 前退出。确认无 solver 进程和无
`g5_schedule` 后，只创建结果父目录并原样启动上述正式命令；这次启动前错误未
被记为 solver attempt，也没有静默参数变化。

长任务按约 30 秒检查 PID、子进程、日志和内存。C2V1 在第二次检查前以 exit 0
完成；G5 在第二次检查前以 exit 2 和完整 schedule manifest 完成。没有 timeout、
kill、重启或隐藏 retry。

## 原始接手目录复核

本轮源码 archive 明确排除 `接手的原始文件/`。本机最终只读复核仍只有三份
接手压缩包，哈希与既有记录一致：

| File | Bytes | SHA-256 |
|---|---:|---|
| `rgbd_safe_method.7z` | 42,052 | `ab2f0a0eceb97d78cb825e04adeac2beb64d27cd3f53a937ff4475708e3893b0` |
| `yzh-skills.7z` | 44,717,933 | `72de126de7a22ef0e6e6b82838710d2dd169822620e1b27fb32d0f886e2eca21` |
| `yzh1-杨子恒的所有原始代码.7z` | 860,136,841 | `405c1a810d72915f14ba759d282e79cc1d8c2d585ee787a591be029e0fe66a07` |

没有在该目录中解压、写入、删除、改名或修改任何文件。

## 后续边界

本轮结果在 `not_ready_at_qSBC_stage1_unknown` 处不可变冻结。后续工作使用新
source snapshot 和 result root，并先建立独立的 G5 数值条件诊断合同，区分：

- 当前 W4/W6 是否在给定 coefficient/epsbar bounds 下真实不可行；
- Putinar 表示、单项式尺度或 Gram 冗余是否导致数值病态；
- 是否需要在不产生形式声明的前提下做 feasibility-only/scale diagnostic。

在新合同冻结前，不应放宽 solver status、把迭代值当作 q、扩大 W 次数，或根据
solver 结果修改已验证 C2 controller。
