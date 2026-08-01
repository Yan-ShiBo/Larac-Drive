# G10R 同-box 冗余乘子消除合同

状态：**IMPLEMENTED, PENDING SERVER STATIC GATES, SOLVE, AND V3 REPLAY**

版本：`G10R-v1`

日期：`2026-08-01`

## 1. 路线边界

G10R 是 G10 旁边的独立求解路线，不替换或修改 `solve-qsbc-g10`。它保持以下内容完全不变：

- M1/S1/P1/D1/N1；
- 冻结 C2 controller 及两个 SHA-256；
- G4 四项原始 q-SBC 义务；
- Chebyshev barrier 空间与系数界；
- `q=nextafter(0.1, 0)`、`epsbar=1e-6` 和 `rho` 目标；
- W14/M2、W16/M2、W16/M4 的有限 schedule 与时间上限；
- V3 exact-rational 独立复核和 fail-closed 发布门禁；
- runtime `monitor_only`，certificate evaluator 不修改动作。

唯一变化是从 LP 中精确消除每个 obligation 上的两个同-box rectangle guard multipliers。X0 initial-disk 与 goal-boundary exterior shape multipliers 继续保留。

## 2. 严格冗余证明

令当前验证 box 为 `B`，其局部坐标为 `s,t in [0,1]`。记 `C_(r,l)(B)` 为在 B 上 `(r,l)` 次 tensor Bernstein 系数逐项非负的多项式锥。

G10 的两个 rectangle guards 是：

```text
g_x = 4 s (1-s)
g_y = 4 t (1-t)
```

若 `B_i^m` 是 m 次一维 Bernstein 基，则有精确恒等式：

```text
B_i^m(s) 4s(1-s)
  = [4 C(m,i) / C(m+2,i+1)] B_(i+1)^(m+2)(s)
```

方括号中的权重严格为正。于是，若 `sigma_x,sigma_y` 的 tensor Bernstein 系数非负，则：

```text
sigma_x g_x in C(B)
sigma_y g_y in C(B)
```

非负 Bernstein 锥对加法、乘法和升阶封闭。对保留 shape guard 后的 core residual 记为 `p`。G10 要求：

```text
r = p - sigma_x g_x - sigma_y g_y - rho in C(B)
```

从任意 G10 可行点出发，设置 `sigma_x=sigma_y=0`，得到：

```text
p - rho = r + sigma_x g_x + sigma_y g_y in C(B)
```

所以同一个 W、rho 和 shape multipliers 在 G10R 中仍可行。反向则令被删乘子为零即可把任意 G10R 可行点嵌回 G10。

因此，在精确算术下：

```text
projection_feasible_set(G10) = feasible_set(G10R)
optimal_rho(G10) = optimal_rho(G10R)
```

这个结论依赖“residual 已经在同一个 B 上做 Bernstein 判正”。它不适用于 initial disk 或 goal exterior：这两个 guard 在各自外接 box 的排除区域内变号，能够真实放宽 box 超集，因此不冗余。

## 3. 冻结规模变化

当前 C2 partition 有 202 个 active drift boxes，其中 28 个 goal-boundary boxes。连同 16 个 X boxes、1 个 X0 obligation 和 3 个 unsafe boxes，共 222 个 obligations。

G10 每个 obligation 分配两个 rectangle multipliers，所以 G10R 精确删除 444 个 multiplier polynomials，只保留 1 个 initial-disk 和 28 个 goal-exterior multipliers。

| stage | G10 variables | G10R variables | reduction |
|---|---:|---:|---:|
| W14/M2 | 4,378 | 382 | 3,996 (91.3%) |
| W16/M2 | 4,411 | 415 | 3,996 (90.6%) |
| W16/M4 | 11,979 | 879 | 11,100 (92.7%) |

不等式行数不变：W14 为 75,150，W16 为 96,798。core barrier 已决定每个 block 的目标 Bernstein 次数，所以删除低次 rectangle columns 不改变升阶次数。

若按精确 Bernstein 支撑计算，被删 columns 的理论非零元数为：

| stage | removed structural nnz |
|---|---:|
| W14/M2 | 934,308 |
| W16/M2 | 1,263,060 |
| W16/M4 | 2,739,300 |

当前 float monomial round-trip 可能把理论零变成微小非零，因此正式性能报告必须引用服务器 manifest 的实际 CSR `nnz`，不能把上表当作实测值。

## 4. 数值尺度审计

Chebyshev 在 `[-1,1]` 上作为函数基是合适的，但转换到 monomial 基并不良态。对 total-degree 变换，静态精确的 1-范数条件数为：

```text
kappa_1(degree 14) = 114243
kappa_1(degree 16) = 665857
```

其中 degree-16 最大列绝对和来自 `T_16`。在 `|c_ij|<=1000` 下，单个 `1000*T_16` 的最高 monomial 系数已经达到 `32,768,000`。这不会破坏 V3 的可靠性，因为 V3 证明最终 binary64 monomial artifact；但它会增加候选转换损失和 exact branch-and-bound 的难度。

原 G10 rectangle multiplier 还有更严重的无意义尺度：goal-boundary 最细 normalized box 约为 `0.04 x 0.02`，M4 local Bernstein 多项式先展开到全局 monomial 再转回局部 Bernstein时，中间系数尺度可达到 `1e15` 量级。这些变量数学上又必然可以取零，因此删除它们同时减少退化和 round-trip cancellation。

保留的 shape multipliers 仍经过同一路径，后续允许的有限数值优化是：

1. 用局部 Bernstein 乘法与升阶直接装配 shape columns，避免全局 monomial round-trip；
2. 用精确整数 Chebyshev 系数和 binary64 `Fraction` 累加最终 monomial W，仅在 artifact 边界舍入一次；
3. 对 LP 行做严格正的对角缩放，或做可逆列重参数化，同时同步 bounds；
4. 在进入 V3 前报告 Chebyshev candidate 与最终 monomial artifact 的转换差异，但不得替代 V3。

这些优化均不得改变 G4 义务、集合、partition、controller、q、epsbar、有限 schedule 或 V3 gate。

## 5. 运行与验收

独立入口：

```text
python -m cbf_rover.entrypoints.research solve-qsbc-g10r \
  --controller <frozen_controller.json> \
  --output-dir <new-empty-directory>
```

本地执行不构成证据。服务器必须先通过定向测试和静态门禁，再用新的只读 source snapshot 运行 G10R。HiGHS status、rho、LP slack 和 shape multiplier 均不是正式证明。

只有以下条件同时成立才可发布 certificate：

```text
V3 status = validated
V3 first_slice_ready = true
V3 acceptance_tolerance = 0
exact_binary64(q) <= 1/10
exact_binary64(epsbar) > 0
all report/tree/controller/barrier hashes cross-bind
```
