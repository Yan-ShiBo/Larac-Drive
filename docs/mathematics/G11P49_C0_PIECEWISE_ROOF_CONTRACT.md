# G11P49 分片 C0 障碍函数与 quadrant-roof 充分证明合同

状态：**CORE IMPLEMENTED; LP, V3P, SERVER GATES, AND SOLVE PENDING**

版本：`G11P49-v1`

日期：`2026-08-01`

## 1. 目标与不变边界

G11P49 是现有全局多项式搜索之外的独立 fallback。它不修改 G4 的无限时域
reach-avoid 定理，也不修改 M1/S1/P1/D1/N1、冻结 C2 controller、`q`、`epsbar`
或 runtime 行为。正式目标仍是找到一个函数 `W`，使：

```text
W >= 0                                      on X
W <= q                                      on X0
W >= 1                                      on Xu
E[W(x_next) | x] <= W(x) - epsbar           on Psi
Psi = X minus (Xg union Xu)
```

runtime 始终为 `monitor_only`；certificate evaluator 只显示和记录障碍证书值，不修改
controller 动作。旧 schema-1/2 certificate 和旧 V3 replay 的语义保持不变。G11P49
只能经独立 V3P 发布 schema-3 certificate。

冻结 ID：

```text
partition  = P49_C2_OBSTACLE_ALIGNED_CARTESIAN_V1
reach      = G11P49_ACTION_DISK_QUADRANT_REACH_CATALOG_V1
proof      = G11P49_C0_PIECEWISE_ROOF_SUFFICIENT_G4_V1
synthesis  = G11P49_C0_BERNSTEIN_ROOF_LP_V1
validator  = G5_EXACT_RATIONAL_PIECEWISE_ROOF_REPLAY_V3P_V1
```

## 2. P49 共形分区与 C0 构造

物理坐标的冻结 break 为：

```text
x = [-1, 0.8, 0.965, 1.915, 2.035, 3.085, 3.2, 4]
y = [-2, -1.385, -0.535, -0.215, 0.215, 0.535, 1.385, 2]
```

对应归一化坐标为：

```text
xi_x = [-1, -7/25, -107/500, 83/500, 107/500, 317/500, 17/25, 1]
xi_y = [-1, -277/400, -107/400, -43/400,
         43/400, 107/400, 277/400, 1]
```

这给出 49 个闭矩形 cell：12 个 unsafe cell、37 个 safe source cell，其中 3 个
与 goal 圆边界相交。内部共有 84 条共形边。障碍函数在每个 cell 上是每轴次数为
`d` 的 tensor Bernstein 多项式，但所有 cell 从一个全局 control net 取系数：

```text
W_ij[a,b] = G[i*d+a, j*d+b].
```

因此相邻 cell 的整条边 control row/column 完全相同，`W` 在整个 `X` 上结构性 C0，
无需浮点连续性等式。`d=3` 时全局自由度为 `22^2=484`；`d=4` 时为
`29^2=841`。在共形边上，任意相邻 piece 的值相同，因此 certificate evaluator
在边界上单值。

C2 的 controller switch `x=0.8` 和 `x=3.2` 也是 P49 break。形式验证对开关两侧
的 source-cell 闭包都建立约束；runtime 的 ordered-first-match 只是其中一个已验证
分支。

## 3. Action-disk reach catalog

对任意 safe source cell `C_i` 和 N1 象限
`sigma in {(-,-),(-,+),(+,-),(+,+)}`，先使用完整 S1 动作圆盘的逐轴外包络，而不是
使用观测到的 C2 动作范围：

```text
normalized action shift:  |Delta xi_x| <= 1/100
                          |Delta xi_y| <= 1/80
normalized N1 half width: a_x = 1/500
                          a_y = 1/400
```

在指定象限加入 `[−a,0]` 或 `[0,a]`，再与 `D1=[-1,1]^2` 相交，得到闭 reach box
`H_i,sigma`。每个 `H_i,sigma` 与所有 49 个 target cell 做正面积相交。冻结 catalog
恰有：

```text
37 source cells * 4 quadrants = 148 roofs
253 target intersections per quadrant
1012 target intersections total
```

零面积共形边无需重复生成 domination row；C0 已保证边界值与任一相邻 target piece
一致。V3P 必须从冻结有理 break 和 S1/N1 常量重新生成 catalog、计数和内容哈希，
不能相信 candidate 随附的 catalog。

## 4. Roof 充分定理

为每个 `(i,sigma)` 引入一个定义在 `H_i,sigma` 上的 tensor-Bernstein 多项式
`R_i,sigma(z)`。对每个正面积交集
`B_i,j,sigma = H_i,sigma intersect C_j` 强制：

```text
R_i,sigma(z) - W_j(z) >= 0       for z in B_i,j,sigma.
```

于是对 `x in C_i`、象限 `sigma` 内任意 N1 扰动，S1 动作圆盘与 D1 invariance 保证
`x_next` 位于某个已覆盖 target closure；C0 保证 target 边界无歧义，因此：

```text
W(x_next) <= R_i,sigma(x_next).
```

N1 每轴独立且为对称三角分布。半区间上的未归一化矩为：

```text
m_n^sigma(a) = sigma^n * a^n / ((n+1)*(n+2)).
```

特别地 `m_0=1/2`，所以二维单象限质量为 `1/4`。令 `L_sigma` 表示将 C2 确定性
successor 代入 roof 后按上述精确矩积分的线性算子，则：

```text
E[W(x_next) | x] <= sum_sigma L_sigma[R_i,sigma](x).
```

因此只要在每个 safe source cell 的 `Psi` 部分验证：

```text
W_i(x) - epsbar - sum_sigma L_sigma[R_i,sigma](x) >= 0,
```

就推出原始 G4 drift obligation。roof 是证明 witness，不是新的系统动力学，也不能
替换最终 G4 结论。

## 5. 凸 LP

冻结 C2 后，`W`、每个 roof、4 个 shape multiplier 和公共 `rho` 都只以系数线性进入
约束。对固定 box 的多项式转换为 tensor Bernstein 系数也是线性的，所以每个 stage
都是凸 LP：

```text
maximize rho
subject to every required Bernstein coefficient >= rho
           coefficient and multiplier bounds.
```

四个 shape multiplier 是：X0 disk 所在 `C_0_3` 上一个，以及三个 goal-boundary
safe cell `C_6_2/C_6_3/C_6_4` 各一个。它们均用非负 Bernstein control coefficients。
unsafe 12 个 cell 与 P49 完全对齐，不需要 unsafe shape multiplier。

有限 schedule：

| stage | W degree | roof degree | multiplier degree | variables | rows | limit |
|---|---:|---:|---:|---:|---:|---:|
| `P49_W3_R4_M2` | 3 | 4 | 2 | 4221 | 27666 | 3600 s |
| `P49_W4_R6_M2` | 4 | 6 | 2 | 8130 | 54121 | 7200 s |
| `P49_W4_R6_M4` | 4 | 6 | 4 | 8194 | 54145 | 14400 s |

系数界为 `W/roof in [-1000,1000]`、shape multiplier in `[0,100000]`、
`rho in [-1,1]`。矩阵必须直接组装为 CSR；禁止生成全局 dense 矩阵。建议 hard cap
为 8 GiB RSS 和 5,000,000 CSR nonzeros。三个 stage 都失败则 `not_ready`，禁止自动
增加次数、改变 partition、修改 controller、修改 `q` 或放宽 `epsbar`。

HiGHS status、numeric `rho`、LP slack 和 multiplier 系数都只是 candidate-generation
evidence，不是形式证明。只有 `rho>0` 的有限数值 candidate 才允许进入 V3P。

## 6. V3P 零容差独立重放

V3P 必须从 schema-3 candidate 的最终 binary64 barrier 和 roof witness 独立重建精确
有理多项式。它不得读取 LP 矩阵、solver status、solver objective、numeric slack 或
shape multiplier。它必须 fail-closed 地验证：

1. P49 break、49/12/37/3 cell 分类、84 条 C0 edge 和 1012 个 reach intersection；
2. barrier、roof witness、geometry、controller、formal-model、`q`、`epsbar` 的双向哈希绑定；
3. `W>=0`、X0、12 个 unsafe cell 的原始 G4 条件；
4. 全部 1012 个 roof domination 条件；
5. 全部 37 个 integrated roof drift 条件，在三个 goal-boundary cell 上保留 goal-exterior 域；
6. C2 controller 的动作圆盘和 robust D1 invariance；
7. `q` 的 exact binary64 值不大于 `1/10`，且 `epsbar` 的 exact binary64 值严格为正。

所有不等式 acceptance tolerance 都是 `0`。若精确 Bernstein subdivision 无法在冻结
资源界内判定，则状态是 `unknown`，并按失败处理，不能把 unknown 当作数值近似通过。

## 7. Schema-3 与事务发布

schema-1/2 继续只接受旧 `SparsePolynomial2D` barrier，并继续由旧 V3 验证。schema-3
显式 dispatch 到 `piecewise_tensor_bernstein_barrier_2d` 和 V3P；禁止通过 payload
猜测把旧 certificate 升级到新语义。

只有 V3P `validated`、`first_slice_ready=true` 且全部哈希门禁通过时，才可以：

1. 写入 `certificate.pending.json`；
2. 再检查发布门禁；
3. 原子替换为 `certificate.json`；
4. 任意异常都删除 pending 和本次创建的 final。

本机静态检查不是运行证据。LP、V3P、测试和正式制品发布均只在服务器进行。
