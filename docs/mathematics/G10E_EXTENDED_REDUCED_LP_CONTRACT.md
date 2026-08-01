# G10E 高阶冗余消除 LP 合同

状态：**IMPLEMENTED, PENDING SERVER STATIC GATES, SOLVE, AND V3 REPLAY**

版本：`G10E-v1`

日期：`2026-08-01`

## 1. 独立路线与冻结边界

G10E 是 G10/G10R 有限 schedule 之外的独立 full-LP fallback，不修改、覆盖或
放宽已有路线。它保持以下内容不变：

- M1/S1/P1/D1/N1 和冻结 C2 controller 的两个 SHA-256；
- G4 四项无限时域 q-SBC 义务；
- `q=nextafter(0.1,0)`、`epsbar=1e-6`、`rho in [-1,1]`；
- Chebyshev W 系数界 `[-1000,1000]`；
- 局部非负 Bernstein shape multiplier 系数界 `[0,100000]`；
- X0 initial-disk 与 goal-exterior shape multipliers；
- G10R 的同-box rectangle multiplier 精确冗余结论；
- V3 exact-rational 独立复核、零 acceptance tolerance 和全部哈希发布门禁；
- runtime `monitor_only`，certificate evaluator 不修改动作。

G10E 唯一新增的搜索空间是以下有限 schedule：

| stage | W degree | multiplier degree | HiGHS limit |
|---|---:|---:|---:|
| `W18_M4` | 18 | 4 | 7200 s |
| `W20_M4` | 20 | 4 | 10800 s |

W18 通过 V3 即停止，不运行 W20。两阶段均失败后状态为 `not_ready`；禁止自动
继续 W22、提高 multiplier 次数、改变 partition 或修改 controller/q/epsbar。

## 2. 凸性

令 total-degree Chebyshev basis 为 `phi_j`，则：

```text
W(xi) = sum_j c_j phi_j(xi)
```

冻结 C2、N1 moments、q 与 epsbar 后，W、`E[W_next]` 和每个 G4 residual 均对
`c_j` 线性。每个保留的 shape guard 是固定多项式，和局部 Bernstein multiplier
相乘后仍只对 multiplier coefficients 线性。固定 box 上的 tensor Bernstein
coefficients 是这些变量的线性函数，rho 也是线性变量。因此每个 stage 都是一个
凸 LP：

```text
maximize rho
subject to affine inequalities and interval bounds
```

HiGHS status、basis、rho、LP slack 和 multiplier values 均只属于候选生成，不能
作为形式证明。

## 3. 同-box rectangle multiplier 的精确消除

G10E 复用 G10R 的恒等式。对局部坐标 `s,t in [0,1]`：

```text
B_i^m(s) 4s(1-s)
  = [4 C(m,i) / C(m+2,i+1)] B_(i+1)^(m+2)(s)
```

权重严格为正，所以非负 tensor-Bernstein multiplier 与 `4s(1-s)` 或
`4t(1-t)` 的乘积仍在同一 box 的非负 Bernstein 锥中。若 legacy residual 为：

```text
r = p - sigma_x g_x - sigma_y g_y - rho in C(B)
```

则：

```text
p-rho = r + sigma_x g_x + sigma_y g_y in C(B)
```

反向令两个 rectangle multipliers 为零即可。因此在精确算术下，删除它们不改变
`(W,rho,shape multipliers)` 的投影可行域或最优 rho。

本路线固定 `m=4`，且 `d=18/20 >= m+2`，core family 决定的目标 Bernstein
次数覆盖被删列的次数。X0 disk 和 goal exterior guard 在外接 box 中变号，不能
使用该论证，故对应 29 个 shape multipliers 全部保留。

G10E 的独立证明 ID 为：

```text
G10E_SAME_BOX_GUARD_PROJECTION_EQUIVALENCE_W18_W20_V1
```

## 4. 冻结规模

当前 partition 有 202 个 active drift boxes，其中 28 个 goal-boundary boxes。
连同 16 个 X boxes、1 个 X0 和 3 个 unsafe boxes，共 222 个 obligations。
G10E 删除 444 个同-box rectangle multiplier polynomials，只保留 29 个 shape
multiplier polynomials。

| stage | W coefficients | shape coefficients | rho | variables | inequalities |
|---|---:|---:|---:|---:|---:|
| W18/M4 | 190 | 725 | 1 | 916 | 121182 |
| W20/M4 | 231 | 725 | 1 | 957 | 148302 |

上述 inequalities 来自当前冻结 controller 的次数结构。正式报告必须使用服务器
manifest 的实际 CSR shape、nnz 和 bytes；静态计数不能替代运行证据。

仅按 `rows * columns * 8` 计算，误建全局 float64 dense 系数矩阵时 W18/M4
约需 0.89 GB，W20/M4 约需 1.14 GB，均未计右端项和求解器副本。按每行最多
包含全部 W/rho 列、且 29 个 shape obligations 再包含 25 个 multiplier 列的
保守计数，最终 CSR 的 data/indices/indptr 上界约为 0.28 GB 和 0.42 GB。实际
该数值假定 32-bit indices；若 SciPy 保留 64-bit indices，对应约为 0.38 GB 和
0.56 GB。实际稀疏度可能更好，但局部稠密 Bernstein 块、临时坐标数组、拼接
和 HiGHS 内部副本会抬高峰值；因此服务器记录还必须包含峰值 RSS，不能从终态
CSR bytes 反推运行峰值。

## 5. 数值风险

高阶 Chebyshev basis 在函数值层面有利，但转成全局 monomial artifact 时条件数
继续增长。W18/W20 的 drift polynomial、CSR nnz、HiGHS 退化和 V3 exact
branch-and-bound 成本均可能明显高于 W16。这些是成功率和资源风险，不是放宽
形式门禁的理由。

必须保留：

- 服务器资源上限、显式 hard timeout 和独立结果目录；
- 最终 Chebyshev candidate 到 monomial artifact 的 binary64 事实记录；
- V3 从最终 monomial W、C2 和 N1 exact moments 从零重建原始 G4 residual；
- `unknown`、`violated`、无 incumbent 或非正 rho 全部 fail closed。

## 6. 入口与发布

独立入口为：

```text
python -m cbf_rover.entrypoints.research solve-qsbc-g10e \
  --controller <frozen_controller.json> \
  --output-dir <new-empty-directory>
```

只有以下条件同时成立才可发布 schema-v2 certificate：

```text
V3 status = validated
V3 first_slice_ready = true
V3 acceptance_tolerance = 0
exact_binary64(q) <= 1/10
exact_binary64(epsbar) > 0
all report/tree/controller/barrier hashes cross-bind
```

本机静态检查不是执行证据；所有 Python、测试、LP 和 V3 均必须在服务器运行。
