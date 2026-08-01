# G10 Chebyshev 多项式乘子稀疏 LP 合同

状态：**IMPLEMENTED, PENDING SERVER SOLVE AND V3 REPLAY**

版本：`G10-v1`

日期：`2026-08-01`

## 1. 目的与声明边界

G10 的唯一验收目标是在不修改 M1/S1/P1/D1/N1、冻结 C2 controller 和 G4
无限时域定理的前提下，构造一个由 V3 独立验证、且风险界严格不大于 `0.10`
的 q-SBC。

G10 不读取 G9 sampled barrier 作为候选或证明，不修改 controller，也不把 exact
M1 的结论转移到 Gazebo 差速模型。runtime 始终为 `monitor_only`，certificate
evaluator 不修改 action。

G10 只接受已经冻结并完成 C2V1 的唯一 controller artifact：

```text
controller content SHA-256 = b0cacedfcd4b884961a786b24e84b2f0f33bbc4ac81aef0b1ff1b0dcfe285984
controller file SHA-256    = df0dbfd2cc5b3aa0a049b5dbb6e7d0360c6b260801e183e95acfb7a9bccf715d
```

ID、decision IDs、content SHA 和 file SHA 必须同时一致。

## 2. 固定 q 与 epsbar

G10 不继续最小化 q，而固定 q 为紧邻 binary64 `0.1` 的下一个较小数：

```text
q decimal       = 0.09999999999999999
q float64 hex   = 0x1.9999999999999p-4
exact(q)        < 1/10

epsbar decimal  = 0.000001
epsbar hex      = 0x1.0c6f7a0b5ed8dp-20
```

固定 q 对“是否存在 `q<=0.10` 证书”没有表达损失：若某个 W 对更小 q 可行，
则同一个 W 对该固定 q 仍可行。使用严格小于 `1/10` 的 binary64 消除了 artifact
中的 `0.1` 在精确有理数语义下略大于 `1/10` 的歧义。

`epsbar` 由 artifact 的 binary64 精确值定义；V3 使用
`Fraction.from_float` 重建该值。

## 3. Barrier basis

在归一化状态 `xi in [-1,1]^2` 上使用 total-degree Chebyshev basis：

```text
W(xi) = sum c_ij T_i(xi_x) T_j(xi_y),  i+j <= d
c_ij in [-1000,1000]
```

综合结束后，Chebyshev W 被确定性转换为 canonical monomial
`SparsePolynomial2D`。V3 只读取最终 monomial W，不读取综合 basis。

## 4. 非负局部多项式乘子

每个局部 box `B=[a,b]x[c,d]` 使用归一化 guard：

```text
gx = 4 (xi_x-a)(b-xi_x)/(b-a)^2
gy = 4 (xi_y-c)(d-xi_y)/(d-c)^2
```

二者在 B 上非负。每个乘子使用 B 上的 tensor Bernstein basis，全部系数满足：

```text
0 <= beta <= 100000
```

因此乘子在整个 B 上非负。对一个待证 residual `p`，G10 要求：

```text
p - sigma_x gx - sigma_y gy - rho >= 0  on B
```

并用全部 tensor Bernstein 系数非负承载最后一个条件。于是 B 上严格推出
`p>=rho`。

X0 外接 box 另加入 initial disk guard：

```text
h0 = r0^2 - ||x-c0||^2
q - W - sigma_0 h0 - sigma_x gx - sigma_y gy - rho >= 0
```

goal-boundary drift box 另加入 goal exterior guard：

```text
gg = ||x-cg||^2 - rg^2
W - E[W_next] - epsbar
  - sigma_g gg - sigma_x gx - sigma_y gy - rho >= 0
```

所有 guard 均固定，因此没有 W 与 multiplier 的乘积。

## 5. 凸性与稀疏实现

冻结 C2 后，解析 N1 期望对 W 系数线性。固定 guard 与 multiplier 相乘时只对
multiplier 系数线性；rho 也线性。因此每个 G10 stage 都是凸 LP：

```text
maximize rho
rho in [-1,1]
```

实现必须把每个 box 的局部 dense Bernstein block 直接追加为 COO，最终转为
CSR。禁止构造全局 dense matrix。manifest 必须记录 CSR 形状、nonzero count
和实际 CSR 字节数。

基于 G8 的固定分区，预计规模为：

| stage | variables | inequalities | CSR nonzeros upper estimate |
|---|---:|---:|---:|
| W14/M2 | 4,378 | 75,150 | 10.5 M |
| W16/M2 | 4,411 | 96,798 | 16.7 M |
| W16/M4 | 11,979 | 96,798 | 20.0 M |

这些数值是执行前估计；正式报告必须引用服务器 manifest 的实际值。

## 6. 有限 schedule

schedule 只有：

| stage | W degree | multiplier degree | HiGHS limit |
|---|---:|---:|---:|
| `W14_M2` | 14 | 2 | 900 s |
| `W16_M2` | 16 | 2 | 3600 s |
| `W16_M4` | 16 | 4 | 7200 s |

每个 stage 固定 q、epsbar、controller、partition 和系数界，只最大化 rho。
HiGHS 的 optimality 不是存在性证明；只要 terminal incumbent 是有限值且
`rho>0`，并且 LP slack 与变量界违例均不超过冻结的 `1e-9` 数值门禁，才允许
进入 V3。`rho<=0`、没有 incumbent、数值可行性门禁失败、V3 `unknown` 或
`violated` 均不能发布 certificate。

schedule 全部失败后，状态固定为 `not_ready`。不得自动提高 W 或 multiplier
次数、扩大系数界、改变 partition 或转入分片 barrier。

## 7. V3 独立验证与正式验收

最终门禁为 `G5_EXACT_RATIONAL_BERNSTEIN_REPLAY_V3`。V3：

- 不读取 HiGHS status、basis 或 LP matrix；
- 不读取任何 G10 multiplier；
- 不读取 G9 sampled residual；
- 将最终 W/controller binary64 系数转成精确有理数；
- 用精确 N1 moments 重新构造原始 G4 residual；
- 在 X、X0 disk、Xu、goal exterior 和 controller switch 两侧闭包上做
  exact-rational Bernstein branch-and-bound；
- 重新验证 C2 的 S1 动作圆盘和 robust D1；
- `acceptance_tolerance=0`，任何预算耗尽均为 `unknown`。
- 报告写入最终 monomial W 的 canonical payload SHA、controller SHA、q/eps 的
  binary64/exact 值，并写入每棵证明树的文件 SHA；certificate 发布前逐项交叉核对。

只有：

```text
V3 status = validated
V3 first_slice_ready = true
exact(q) <= 1/10
exact(epsbar) > 0
```

才发布 `ready` certificate。此时 G4 推出：

```text
P(hit Xu before Xg) <= q < 0.10
P(hit Xg before Xu) >= 1-q > 0.90
E[tau] <= q/epsbar
```

certificate schema v2 同时保存上述派生界的精确有理数字符串。用于显示的概率下界
必须向负无穷方向舍入，停止时间上界必须向正无穷方向舍入，禁止普通 binary64
四舍五入把严格下界抬高或把严格上界压低。

服务器求解和 V3 尚未完成前，G10 状态只能是 `pending` 或 `not_ready`，不能
写成障碍证书证明已经完成。
