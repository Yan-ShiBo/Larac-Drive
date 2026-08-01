# G7 高阶 Bernstein LP 证书综合合同

状态：**CONFIRMED ACTIVE**

版本：`G7-v1`

日期：`2026-07-30`

## 1. 触发证据

固定 C2 下，G5 与局部坐标 G6 的 W4/W6 均在第一目标返回 `Unknown`。诊断专用
DX2 使用同一 controller、同一解析 N1 期望和 9523 点几何网格，得到：

| W degree | sampled q, epsbar=1e-6 |
|---:|---:|
| 4 | infeasible |
| 6 | infeasible |
| 8 | 0.7768396032 |
| 10 | 0.4784692184 |
| 12 | 0.2109737878 |
| 14 | 0.0916013617 |
| 16 | 0.0650843989 |

DX2 不是形式证据，但明确支持把“低阶全局 W 表达不足”作为下一步假设。直接把
Putinar SDP 提高到 14/16 次会产生过大的 Gram 矩阵，因此 G7 改用可扩展的
Bernstein 系数充分条件。

## 2. 固定数学义务

G4 theorem、C2、M1/P1/D1/N1、解析期望和全局 W 定义不变。G7 schedule 只有：

```text
W total degree: 14 -> 16
epsbar: 1e-6
W coefficient bounds: [-1000,1000]
objective: minimize q
q bounds: [0,1]
```

不得自动升到 18 次。

## 3. 线性充分条件

对固定 box，global monomial W 或 drift residual 的 tensor Bernstein 系数是 W
系数的线性函数。G7 对每个覆盖 box 强制所有 Bernstein 系数至少为
`2e-7`，因此由 Bernstein convex-hull property 得到整个 box 上的非负性。

- `W>=0 on X`：把 `[-1,1]^2` 均匀分成 4 x 4；
- `W<=q on X0`：在 X0 的外接矩形上验证，强于圆盘义务；
- `W>=1 on Xu`：对三个 unsafe rectangle 分别验证；
- drift：从 obstacle/controller-aligned formal cells 出发，每格再分成 2 x 2；
- goal disk：完全位于 goal 内的 box 删除；相交 box 最多递归 12 层，最后保留
  的 boundary box 是 goal-exterior 的保守超集；若任何活动 box 仍包含 goal
  center，综合器 fail closed。

因此 G7 的 LP 约束域不小于 G4 所需域。保守超集可能造成不可行，但不会制造
虚假证书。

## 4. 两层独立性

HiGHS optimal 只产生 numeric candidate。随后
`G5_EXACT_FLOAT_BERNSTEIN_REPLAY_V2` 从最终 binary64 W、q、epsbar 和 C2
重新构造原始全局不等式：

- 不读取 G7 Bernstein partition matrix 或 LP slack；
- 不读取 HiGHS basis/status 作为证明；
- 使用 exact binary64-to-rational 转换与向外舍入；
- 在原 disk goal、disk X0 和原 formal cells 上验证；
- `unknown` 或 `violated` fail closed。

只有独立 replay 为 `validated` 才发布 certificate；只有 `q<=0.10` 才标为
`ready`。runtime 只加载最终 certificate，不加载 DX2 或 G7 临时候选。
