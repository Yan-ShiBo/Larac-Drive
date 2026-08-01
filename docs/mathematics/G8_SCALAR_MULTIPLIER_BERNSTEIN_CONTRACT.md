# G8 Bernstein 标量乘子 q-SBC 综合合同

状态：**CONFIRMED ACTIVE**

版本：`G8-v1`

日期：`2026-07-30`

## 1. 触发证据

G7 的 W14 候选通过了独立全域复核，但 `q=0.8765194159459675`，仅为
`formal_valid_not_ready`。W16 在 600 秒 HiGHS 限时内返回
`Time limit reached`，没有 numeric candidate。

W14 的 G7 LP 含 246150 条不等式，其中 189450 条来自 goal-capture 区域的
保守 boundary-box 超集。DX2 在原集合的密集采样上得到 W14
`q=0.0916013617`。因此下一项可检验假设是：主要松弛损失来自“把 goal 圆盘外部
替换为大量矩形超集”，而不是 W14 本身没有表达能力。

## 2. 不变项

以下内容全部冻结：

- M1/S1/P1/D1/N1；
- C2 controller 的系数、分段闭区间语义与内容哈希；
- G4 无限时域 q-SBC 定理和四项原始不等式；
- `epsbar=1e-6`、W total degree 14、W 系数界 `[-1000,1000]`；
- objective 为最小化 `q`，`q in [0,1]`；
- 独立复核器 `G5_EXACT_FLOAT_BERNSTEIN_REPLAY_V2`；
- `q<=0.10` 才能成为 `first_slice_ready`；
- runtime 始终为 `monitor_only`。

本轮不自动提高 W 次数，不修改 controller，不改变集合，也不读取 G7 的 W14
作为 warm start。

## 3. 精确集合蕴含

记 initial disk guard

```text
h0(x) = r0^2 - ||x-c0||^2
```

在 `X0` 上有 `h0>=0`。G8 为 initial obligation 引入一个标量
`lambda0>=0`，并在 X0 外接小矩形上用 Bernstein 系数证明：

```text
q - W - lambda0*h0 >= 2e-7
```

因此在原始圆盘 X0 上严格推出 `q-W>=2e-7`，不再要求矩形四角也满足
`W<=q`。

记 goal-exterior guard

```text
gg(x) = ||x-cg||^2 - rg^2
```

在 `X\Xg` 上有 `gg>=0`。对每个与 goal 边界相交的 drift box，G8 引入独立
标量 `lambda_D>=0`，并证明：

```text
W - E[W_next] - epsbar - lambda_D*gg >= 2e-7
```

于是只在原始 goal 外部推出 G4 drift。完全位于 goal 内的 box 不施加 drift；
完全位于 goal 外的 box 直接证明 drift。每个 formal cell 先做一次 2x2 均匀
细分，goal boundary box 再做最多两层 2x2 细分。

所有乘子只参与凸 LP，取值界为 `[0,100000]`。该上界是明确的数值门禁，不是
定理假设。

## 4. 其余充分条件

- `W>=0 on X`：归一化域按 4x4 boxes 覆盖；
- `W>=1 on Xu`：三个原始 unsafe rectangles 分别覆盖；
- 每个正性条件要求全部 tensor Bernstein 系数至少为 `2e-7`；
- solver 为 SciPy HiGHS LP，primal/dual feasibility tolerance 均为 `1e-9`；
- solver time limit 为 900 秒；
- 非 optimal、timeout、异常均记为 `infeasible_or_unknown`。

## 5. 独立复核与停止规则

HiGHS 只能产生 numeric candidate。独立复核器从最终 binary64
`W/q/epsbar` 和冻结 C2 重新构造原始 G4 多项式：

- 不读取任何 G8 标量乘子；
- 不读取 LP 矩阵、basis 或 solver status 作为证明；
- 在原始 X0 disk、Xg disk、Xu rectangles 和 controller-aligned drift cells
  上用 exact-float rational Bernstein branch-and-bound；
- `unknown` 或 `violated` 均 fail closed。

仅当独立复核为 `validated` 且 `q<=0.10` 时发布 `ready` certificate。若本轮
失败，先保存完整 manifest，再依据具体约束与乘子边界诊断；禁止静默重跑或
自动提高次数。
