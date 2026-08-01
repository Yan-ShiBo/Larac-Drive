# G4 无限时域 reach-avoid q-SBC 定理证明

状态：**定理证明已冻结；数值见证等待 V3 零容差复核**

版本：`G4-proof-v1`

日期：`2026-08-01`

## 1. 适用对象

令 `x_k` 为 M1/S1 的闭环状态，控制为冻结的确定性 Markov feedback
`u_k = pi(x_k)`，噪声为 N1 两轴独立、时间 IID 的三角分布。令

```text
tau = inf{k >= 0 : x_k in Xg union Xu}.
```

本证明只适用于与证书工件完全同一身份的 M1/S1/P1/D1/N1、归一化、controller
和集合。Gazebo 差速动力学、随机生成障碍、tracking error 和在线 CBF 不继承本
定理的概率界。

## 2. 显式前提

除四项 q-SBC 多项式义务外，发布证书前还必须独立验证：

1. `X0 subset X`、`Xg subset X`、`Xu subset X`；
2. `X0` 与 `Xg union Xu` 不相交，且 `Xg` 与 `Xu` 不相交；
3. `pi` 是在 `X` 上全定义、确定、Borel 可测的 Markov feedback；
4. controller 输出直接进入 M1，不存在证书反馈或后置 clipping；
5. N1 的两轴独立且相对自然滤过时间 IID，其 support 与解析矩一致；
6. 对所有 `x in closure(Psi)` 和所有 N1 support 中的 `w`，后继状态仍在 `X`；
7. `W`、`q`、`epsbar`、controller 和模型合同的内容哈希与复核输入一致。

其中 `Psi = X \ (Xg union Xu)`。C2 的动作圆盘与 D1 验证是第 4、6 项的组成
部分，不由 q-SBC 残差自动替代。

## 3. q-SBC 义务

假设存在连续函数 `W : X -> R`、`q in [0,1)` 和 `epsbar > 0`，满足

```text
W(x) >= 0                                      for x in X,
W(x) <= q                                      for x in X0,
W(x) >= 1                                      for x in Xu,
E[W(x_{k+1}) | x_k=x] <= W(x)-epsbar          for x in Psi.
```

当前工程中的 `W` 是归一化状态上的有限多项式，因此在紧集 `X` 上连续且有界。
期望必须由 N1 的解析矩精确构造；Monte Carlo 或密网格不能替代第四项。

## 4. 超鞅论证

令 `F_k` 为状态和噪声生成的自然滤过，并定义停止过程

```text
Z_n = W(x_{n wedge tau}) + epsbar (n wedge tau).
```

当 `n < tau` 时，`x_n in Psi`，第四项给出

```text
E[Z_{n+1} | F_n] <= Z_n.
```

当 `n >= tau` 时，停止过程保持不变。因此 `(Z_n)` 是可积超鞅，且对每个
`x_0 in X0`、每个有限 `n`，

```text
E[W(x_{n wedge tau})] + epsbar E[n wedge tau]
    <= W(x_0)
    <= q.
```

D1 保证停止前后所用状态均位于 `X`，第一项又给出
`W(x_{n wedge tau}) >= 0`。于是

```text
epsbar E[n wedge tau] <= q.
```

由单调收敛定理，

```text
E[tau] <= q / epsbar < infinity.
```

因此 `tau < infinity` 几乎必然。

## 5. reach-avoid 概率结论

由于 `W` 在紧集 `X` 上有界，`W(x_{n wedge tau})` 可用支配收敛取极限。得到

```text
E[W(x_tau)] + epsbar E[tau] <= W(x_0) <= q.
```

在事件 `A = {先到 Xu}` 上，第三项给出 `W(x_tau) >= 1`；在互斥的目标终止
事件上，第一项给出 `W(x_tau) >= 0`。因此

```text
P(A) <= E[W(x_tau)] <= q,
P(先到 Xg) = 1-P(A) >= 1-q,
E[tau] <= q/epsbar.
```

这里 `q` 是对所有 `x_0 in X0` 成立的无限时域风险上界，不是逐帧碰撞概率；
`1-W(x_k)` 也不自动具有即时成功概率含义。

## 6. 数值见证的发布门禁

定理成立不等于某个数值多项式已经满足定理。一个可发布的见证必须同时通过：

- 全链精确有理数或严格有向区间的期望与残差重构；
- 圆盘、矩形、controller switch 两侧闭包的保守域分类；
- 原始四项不等式的零容差全域验证；
- C2 动作圆盘与 robust D1 的同等级独立重放；
- `q < 1`、`epsbar > 0` 以及完整模型/controller/验证报告哈希绑定。

项目的首片完成门槛还要求 `q <= 0.10`。`q > 0.10` 的严格见证只能标为
`formal_valid_not_ready`；任何 `unknown`、负容差接受或模型哈希不匹配都必须
fail closed。
