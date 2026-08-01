# G6 局部坐标 SOS 数值合同

状态：**CONFIRMED ACTIVE**

版本：`G6-v1`

日期：`2026-07-30`

## 1. 触发条件

固定 C2 controller 已通过 C2V1。原 G5 在 W4、W6 的第一目标均返回
`SolutionStatus.Unknown`；日志同时出现大范数和不可接受的最终 constraint
violation。由于每个 drift/unsafe cell 在全局 `xi` 单项式中很窄，原 Putinar
guard 的系数尺度差异会迫使 Gram 表示产生很大的乘子。

G6 只修正 SOS 表示的数值坐标，不改变 G4 theorem、C2、集合、噪声、W basis、
次数 schedule、变量、系数 bound、目标或独立验证门禁。

## 2. 局部仿射 pullback

对每个闭矩形约束域

```text
B = [a_x,b_x] x [a_y,b_y]
```

引入局部变量 `z in [-1,1]^2`：

```text
xi_x = (a_x+b_x)/2 + (b_x-a_x) z_x/2
xi_y = (a_y+b_y)/2 + (b_y-a_y) z_y/2
```

将待证多项式和非矩形 guard 精确做仿射代换，再使用固定矩形 guard：

```text
1-z_x^2 >= 0
1-z_y^2 >= 0
```

`X0` 单独映射为单位圆盘，使用：

```text
1-z_x^2-z_y^2 >= 0
```

goal-exterior guard 在每个 drift cell 中做相同 pullback，并只除以一个严格正的
系数尺度；正缩放不改变其符号或集合。

## 3. 保持不变的数学问题

- W 仍是全局 `xi` total-degree 4 或 6 多项式；
- C2 piece、switch、动作和哈希完全只读；
- exact N1 moment expansion 不变；
- q、epsbar、W coefficients 和 SOS Gram matrices 仍是全部变量；
- W coefficient bound 仍为 `[-1000,1000]`；
- `epsbar in [1e-6,1]`；
- 仍先最小化 q，再在 `q<=q_star+1e-7` 下最大化 epsbar；
- 每个 objective 仍为 600 秒、8 线程；
- schedule 仍只有 `4 -> 6`。

仿射代换对固定 cell 是线性的，guard 正缩放也是固定数值，因此问题仍为凸 SDP。

## 4. 独立验证与声明

solver 最终值仍必须由
`G5_EXACT_FLOAT_BERNSTEIN_REPLAY_V2` 在原全局 `xi` 坐标中重新构造和验证。
validator 不读取局部 Gram matrix、SOS multiplier、solver status 或 PTF。

只有：

```text
solver status = Optimal
independent replay = validated
q < 1
epsbar > 0
```

才是 `formal_valid`；还需 `q<=0.10` 才是首片 `ready`。`Unknown`、数值候选
replay 失败或 W4/W6 全部失败都 fail closed，不能放宽为成功。
