# C2 分段多项式 controller 合同

状态：**FROZEN CANDIDATE, PENDING SERVER C2V1**

版本：`C2-v1`

日期：`2026-07-30`

## 1. 采用原因与证据边界

首轮正式服务器运行选择了 V1 seed 11，但 PB1 的三个既定稀疏 support 阶段均
未通过独立 selection gate。随后只读诊断 DX1 证明：

- total degree 1 至 6 的 dense 全局多项式均无法达到旧 PB1 的
  `0.05 m/s` 全局模仿阈值；
- reference action 在全域呈现陡峭、局部多模态结构；
- seed 11 从 `X0` 出发的诊断轨迹全部选择下方通道。

DX1 不是安全证明，也不直接提供 C2 系数。它只支持停止“继续增加全局 PAC
次数/项数”，转而冻结一个与 Rover 几何一致的低路由 controller。C2 系数在
任何 q-SBC 求解前确定；证书、`W`、`q`、`epsbar` 和 SOS 结果均不参与 C2
构造或调参。

## 2. 固定状态、动作与归一化

C2 继续使用 M1/S1/P1/D1/N1，不改变系统或集合：

```text
xi_x = (x_m - 1.5) / 2.5
xi_y = y_m / 2.0
u = [u_x_mps, u_y_mps]
```

动作输出后不允许 clipping、projection、rescaling 或 certificate feedback。

## 3. 闭区间分区与固定系数

分区变量为物理坐标 `x_m`：

```text
I1 = [-1.0, 0.8]
I2 = [ 0.8, 3.2]
I3 = [ 3.2, 4.0]
```

各段 controller 为：

```text
I1 / lower_entry:
  u_x = -0.05 - 0.25 xi_x
  u_y = -0.28 - 0.11 xi_y + 0.28 xi_y^2

I2 / lower_transit:
  u_x =  0.18 - 0.25 xi_x
  u_y = -0.26 - 0.11 xi_y + 0.26 xi_y^2

I3 / goal_capture:
  u_x =  0.42 - 0.525 xi_x
  u_y = -0.20 xi_y
```

前两段把轨迹引向并维持在下方通道；最后一段同时收敛到
`(3.5 m, 0 m)`。这些解释只是设计意图，不能代替 C2V1 或 q-SBC。

## 4. 共享边界语义

运行时使用按 artifact 顺序的闭区间首个匹配：

```text
x = 0.8  -> lower_entry
x = 3.2  -> lower_transit
```

形式验证采用更保守的规则：每个相邻 piece 都在自己的闭包上验证。因此
`x=0.8` 和 `x=3.2` 的左右多项式都必须满足动作与 D1 gate；q-SBC drift
分解也在边界两侧的闭 cell 上分别施加约束。该处理不要求 controller 连续，
但固定了一个可审计、可测量的分段 Markov feedback。

## 5. C2V1 冻结门禁

对每个 piece 的整个闭区间与全部 `y in [-2,2]`，用 binary64 精确转换和向外
舍入 Bernstein 验证：

```text
0.25 - u_x^2 - u_y^2 >= 1e-6
```

对每个 piece 与 `closure(Psi)` 的交集，消去 N1 support 后验证：

```text
x + 0.05 u_x >= -0.995
x + 0.05 u_x <=  3.995
y + 0.05 u_y >= -1.995
y + 0.05 u_y <=  1.995
```

每个残差要求至少 `1e-6 m`。任何 `unknown` 或 `violated` 都 fail closed。

形式 gate 通过后，运行 10 个固定 base seed、每 seed 100 个 `X0` episode、
每个最多 2000 step 的 exact 诊断。要求：

```text
goal_rate >= 0.99
unsafe/domain/action/numerical hard failures = 0
```

rollout 只作为经验门禁。只有形式 gate 和 rollout gate 同时通过，才生成
`frozen_controller.json`。

## 6. Artifact 与凸性

C2 使用 `frozen_piecewise_polynomial_controller` schema v2，内容哈希覆盖：

- 三个 piece 的闭区间、稀疏指数对和 binary64 系数；
- 归一化、frame、动作顺序和共享边界语义；
- C2 合同、DX1 内容哈希与文件哈希；
- `monitor_only` 和禁止后置 clipping 的语义。

q-SBC 阶段将整个 C2 artifact 作为只读数值输入。每个 drift cell 只使用覆盖
其内部的固定 piece，并在 switch 两侧分别建立约束。优化变量仍只有：

```text
W coefficients, q, epsbar, SOS Gram matrices
```

因此固定 C2 后的 G5 仍是凸 SDP；不允许联合优化 controller 与 certificate。

## 7. 声明边界与停止条件

- C2V1 通过只证明动作圆盘与 robust D1 域不变性，并支持冻结；它不证明避障。
- exact rollout 成功不构成概率证明。
- 只有 G4 全部不等式通过独立 G5 replay，才能形成 q-SBC 形式声明。
- Gazebo 差速跟踪仍是经验映射，不继承 exact 证书，除非未来另行证明 tracking
  error contract。
- C2V1 失败时状态为 `not_ready_at_C2V1`，不得调高多项式次数或用证书结果反向
  修改 C2。
- G5 的 `W degree 4 -> 6` 全部失败时状态为 `not_ready_at_qSBC`，不得自动
  增加次数。
