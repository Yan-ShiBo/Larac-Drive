# Rover-first q-SBC 研究合同

状态：**CONFIRMED**

版本：`contract-v4`

确认日期：`2026-07-30`

确认方式：用户授权“后续所有选择均采用推荐项，所有测试与运行均在服务器执行”。

本文件是首个 q-SBC vertical slice 的唯一数学合同。代码、配置、训练、
controller、证书和报告只要与本合同的语义或内容哈希不一致，就必须 fail closed。

## 1. 研究问题与边界

首片只回答：

> 对固定的二维随机 single-integrator、固定的 Rover 投影集合、固定的确定性
> 多项式 controller 和 N1 IID 三角噪声，能否构造并独立验证一个无限时域
> reach-avoid q-SBC？

首片不声称：

- SAC 是安全策略；
- PAC 误差本身给出安全概率；
- `1-W(x)` 是即时成功概率；
- 100 秒 rollout 是形式定理的时间范围；
- Gazebo 差速跟踪自动继承精确模型的形式结论；
- 当前噪声、footprint 或 tracking error 来自实车标定。

## 2. M1/S1 系统

决策 ID：`M1_STOCHASTIC_SINGLE_INTEGRATOR_2D`、
`S1_DT005_SPEED_DISK05_NO_POST_CLIP`。

状态和控制均在 `world` frame：

```text
x[k] = [x_m, y_m]
u[k] = [u_x_mps, u_y_mps]
x[k+1] = x[k] + 0.05 u[k] + w[k]
U = {u : u_x^2 + u_y^2 <= 0.25}
```

controller 输出后禁止 clipping、projection、wrap 或未入哈希的 rescaling。
`max_steps=2000` 只定义 100 秒诊断仿真，不定义证书时域。

## 3. P1/D1 集合

决策 ID：`P1_CONSERVATIVE_INFLATED_AABB`、
`D1_COMPACT_ROBUST_FORWARD_INVARIANT`。

```text
X  = [-1,4] x [-2,2]
X0 = {(x,y): x^2+y^2 <= 0.10^2}
Xg = {(x,y): (x-3.5)^2+y^2 <= 0.25^2}

Xu1 = [0.965,2.035] x [-0.535, 0.535]
Xu2 = [1.915,3.085] x [ 0.215, 1.385]
Xu3 = [1.915,3.085] x [-1.385,-0.215]
Xu  = Xu1 union Xu2 union Xu3
Psi = X \ (Xg union Xu)
```

每个 `Xui` 都是物理 box 与半径 `0.31+0.05=0.36 m` 圆盘的 Minkowski
sum 的 conservative AABB，因此是物理膨胀障碍的超集。

对所有 `x∈closure(Psi)` 和所有 N1 support 中的 `w`，冻结 controller 必须满足：

```text
x + 0.05 u(x) + w in X
```

超出 `X` 的 successor 是 `model_contract_breach`；禁止继续轨迹或把它计入任何
certificate-supported 统计。

## 4. N1 噪声与精确期望

决策 ID：`N1_IID_SYMMETRIC_TRIANGULAR_5MM`。

```text
w_x,w_y ~ Triangular(-0.005,0,0.005) m
```

两个轴相互独立，各时间步 IID。令 `a=0.005`，单轴矩为：

```text
E[w^r] = 0                                  r 为奇数
E[w^r] = 2 a^r / ((r+1)(r+2))              r 为偶数
```

形式残差必须使用解析矩展开，不允许 Monte Carlo 期望。采样只能用于经验诊断。

## 5. K2/T2/V1 reference policy

决策 ID：`K2_SAC_REFERENCE_TO_PAC_POLYNOMIAL`、
`T2_SYMMETRIC_UNBIASED_SAC`、`V1_SELECT_ONE_SAC_BEFORE_PAC`、
`ST1_SAC_600K_SYMMETRIC_GLOBAL_V1`。

训练五个互相独立的 SAC seed：

```text
[11,23,47,89,131]
```

每个 seed 训练 `600000` steps，网络为 `[256,256]`。训练 reset 由 `30% X0`
和 `70% Uniform(Psi)` 混合；reward 只包含对称的 goal progress、control、
clearance、boundary 和 terminal 项，不包含上/下 waypoint、route label 或路线偏置。

SAC 原始 `[-1,1]^2` action 通过 `ST1_CONCENTRIC_SQUARE_TO_DISK_042` 映射到
半径 `0.42 m/s` 的 reference disk。该 codec 只属于 SAC 环境，不属于最终
controller。

每个 checkpoint 使用 deterministic mean action，在五个独立 eval seed
`[1009,2027,3037,4051,5059]` 上各跑 20 个 episode。hard gate 要求：

```text
unsafe_entry = 0
model_contract_breach = 0
action_contract_breach = 0
```

通过者依次按 goal rate 降序、成功 episode median steps 升序、训练 seed
升序选出唯一 checkpoint。没有通过者则 `not_ready`。选择过程禁止读取 PAC、
CV1、q-SBC 或 Gazebo 结果。

## 6. PB1/PA1/PS1 多项式 controller

归一化：

```text
xi_x = (x-1.5)/2.5
xi_y = y/2
```

显式 monomial catalog：

```text
M_d = {xi_x^i xi_y^j : i,j>=0, i+j<=d}
```

顺序为 total degree 升序、同阶 `i` 升序。`u_x/u_y` 独立 support，artifact
以指数对定义 support identity。

| 阶段 | catalog 最高次数 | 每轴最多项数 |
|---|---:|---:|
| 1 | 3 | 6 |
| 2 | 3 | 8 |
| 3 | 4 | 8 |

三阶段全部失败即 `not_ready`，不自动增加次数或项数。

PAC 设计分布为 `Uniform(Psi)`，通过从 `Uniform(X)` rejection sampling
得到。每阶段数据严格隔离：

| 用途 | 样本数 |
|---|---:|
| support search | 50000 |
| support selection | 20000 |
| PAC refit | `max(5000,ceil(2/epsilon*(ln(1/eta_s)+kappa)))` |
| boundary stress | 独立诊断 |

其中：

```text
epsilon_pac = 0.05
eta_total = 0.001
eta_s = eta_total/3
kappa = 1+|Sx|+|Sy|
e_star <= 0.05 m/s
```

support search 使用 deterministic backward-elimination `L_inf` beam search，
`beam_width=32`，每个临时 support 都用 HiGHS LP。候选 pair 在 selection
集上按 pointwise `L_inf`、pointwise mean、总项数、最大次数、exponent tuple
排序。每阶段最多一次正式 PAC refit；search/selection 临时系数随后丢弃。

PAC 声明仅是 SAC-to-polynomial imitation 声明，不是安全概率、D1 证明或 q-SBC
slack。

### 6.1 PB1 正式结果与 DX1 分支决策

正式服务器运行中，PB1 三个有限阶段均未通过独立 selection gate，状态冻结为
`not_ready_at_PB1_PAC`。DX1 随后只读诊断了 seed 11 与三个失败 manifest，
确认继续增加全局 polynomial support 不适合作为下一步；DX1 本身不是安全证明。

活动路线转为决策 ID `C2_FIXED_LOWER_PIECEWISE_POLYNOMIAL_V1`：在证书求解前
冻结三段低路由多项式 controller。精确区间、系数、共享边界和验证规则见
[`C2_PIECEWISE_CONTROLLER_CONTRACT.md`](C2_PIECEWISE_CONTROLLER_CONTRACT.md)。
不得复用失败 PB1 系数，不得根据 q-SBC 结果反向修改 C2。

## 7. C1/CV1/C2V1 controller 冻结

决策 ID：`C1_FREEZE_CONTROLLER_THEN_CERTIFY`、
`CV1_BERNSTEIN_GLOBAL_AND_1000_ROLLOUTS`、
`C2V1_PIECEWISE_BERNSTEIN_AND_1000_ROLLOUTS`。

全局 PB1 候选在 `X` 上、C2 候选在每个 piece 的闭区间上必须独立通过：

```text
0.25-u_x(x)^2-u_y(x)^2 >= 1e-6              x in X
```

消去 N1 bounded support 后，D1 等价于在 `closure(Psi)` 上验证：

```text
x+0.05u_x >= -0.995
x+0.05u_x <=  3.995
y+0.05u_y >= -1.995
y+0.05u_y <=  1.995
```

每个残差 margin 至少 `1e-6 m`。验证使用对 artifact 中 binary64 系数精确
转换、向外舍入的 Bernstein branch-and-bound；保存 subdivision tree。任何
`unknown` 均 fail closed。

形式 gate 通过后，再运行 10 个固定 seed、每 seed 100 episode、最多 2000
step 的 exact 诊断。要求零 unsafe/domain/action/numerical failure 且 goal rate
至少 `0.99`。rollout 不是形式证明。

通过后冻结 controller 系数与语义哈希。q-SBC 阶段只读该 artifact，只优化
`W,q,epsbar` 和 SOS multipliers。runtime 为 `monitor_only`，evaluator 不修改
action。

## 8. G4 无限时域 q-SBC

决策 ID：`G4_QSBC_INFINITE_HORIZON_REACH_AVOID_V1`。

令：

```text
tau = inf{k>=0 : x[k] in Xg union Xu}
```

如果存在多项式 `W`、`q∈[0,1)` 和 `epsbar>0`，使：

```text
W(x) >= 0                                      x in X
W(x) <= q                                      x in X0
W(x) >= 1                                      x in Xu
E[W(x[k+1]) | x[k]=x] <= W(x)-epsbar          x in Psi
```

并且 M1、N1、D1 和冻结 controller 合同均成立，则对所有 `x0∈X0`：

```text
P(hit Xu before Xg) <= q
P(hit Xg before Xu) >= 1-q
E[tau] <= q/epsbar
```

严格 drift 给出有限 expected stopping time，因此 termination 几乎必然。
`q` 是初始集合上的无限时域 reach-avoid risk upper bound，不是单步或逐状态
即时概率。`epsbar` 是 drift margin，不得改名为安全概率。

## 9. G5/G6 求解和独立验证

求解 ID：`G5_PUTINAR_MOSEK_DEGREE_4_6_V1`。

- 变量：归一化 total-degree `W` 系数、`q`、`epsbar`、SOS Gram matrices；
- 固定输入：controller artifact、集合、N1 moments；
- 求解：MOSEK Fusion convex SDP；
- `W` 次数 schedule：`4 -> 6`，不得自动继续；
- `W` 系数 bounds：`[-1000,1000]`；
- `epsbar∈[1e-6,1]`；
- 先最小化 `q`，再在 `q<=q_star+1e-7` 下最大化 `epsbar`；
- 每个 objective time limit 为 600 秒；
- infeasible、non-optimal、timeout、异常均为 `infeasible_or_unknown`。

历史 validator `G5_EXACT_FLOAT_BERNSTEIN_REPLAY_V2` 使用 float 展开和
`1e-7` acceptance tolerance，不能再作为严格形式证据。活动 validator ID 为
`G5_EXACT_RATIONAL_BERNSTEIN_REPLAY_V3`。

V3 从最终 `W`、controller 和精确 N1 moments 重新构造所有残差，使用 exact
rational Bernstein branch-and-bound；分段 controller 的 drift cell 在每个
switch 处拆开并覆盖相邻两个闭包。它不读取 solver status、SOS multiplier、
Gram matrix 或 sampled residual，正式 acceptance tolerance 固定为零，`unknown`
fail closed。报告必须绑定 W/controller/q/epsbar 与每棵证明树的 SHA-256。

两级状态必须分开：

- `formal_valid`：validator 通过、`q<1`、`epsbar>0`；
- `first_slice_ready`：在上述条件上再要求 `q<=0.10`。

形式有效但 `q>0.10` 的产物可以作为审计候选保留，不能标成首片完成。

### 9.1 G5 结果与 G6 局部坐标改进

固定 C2 的正式 G5 运行中，W4 和 W6 均在第一目标返回
`SolutionStatus.Unknown`，未产生 numeric candidate 或证书。该结果冻结为
`not_ready_at_qSBC_stage1_unknown`，不能解释为数学不可行。

活动求解 ID 转为 `G6_LOCAL_COORDINATE_PUTINAR_MOSEK_V1`。G6 对每个
rectangle、initial disk 和 drift cell 把固定全局多项式义务仿射 pullback 到
局部单位 box/disk，并对 goal guard 只做正系数缩放。W、controller、G4 不等式、
优化变量、bounds、目标、degree schedule 与独立全局 validator 均不变。完整
数值合同见
[`G6_LOCAL_COORDINATE_SOS_CONTRACT.md`](G6_LOCAL_COORDINATE_SOS_CONTRACT.md)。

### 9.2 G6 结果与 G7 高阶综合

G6 改善了 W4 的 primal 数值尺度，但 W4/W6 仍为 `Unknown`。DX2 sampled LP
显示正式 `epsbar=1e-6` 从 W8 开始可行，并在 W14/W16 首次得到诊断
`q<=0.10`。活动综合 ID 因此转为
`G7_BERNSTEIN_PARTITION_HIGHS_DEGREE_14_16_V1`。

G7 用分区 Bernstein 系数非负这一线性充分条件承载 W14/W16；X0 使用外接
rectangle，goal boundary box 使用保守超集。它不放宽任何 G4 原义务。最终候选
仍必须由 `G5_EXACT_RATIONAL_BERNSTEIN_REPLAY_V3` 在原集合上独立验证。完整合同见
[`G7_BERNSTEIN_LP_CONTRACT.md`](G7_BERNSTEIN_LP_CONTRACT.md)。

### 9.3 G7 结果与 G8 标量乘子改进

G7 的 W14 候选经 V3 零容差复核后通过全部多项式、controller 和几何门禁，但
`q=0.8765194159459675`，状态为 `formal_valid_not_ready`；W16 在 600 秒内返回
`Time limit reached`，没有候选。原 V2 报告仅作为历史工件，不再承担证明作用。
W14 的 246150 条 LP 不等式中有 189450 条来自 goal boundary 的矩形保守超集，
与 DX2 的 W14 sampled `q=0.0916013617` 形成明确差异。

活动综合 ID 转为 `G8_BERNSTEIN_SCALAR_MULTIPLIER_LP_W14_V1`。G8 固定 W14 与
`epsbar=1e-6`，对 X0 disk 和 goal exterior 分别加入非负标量 S-procedure
乘子，以线性 Bernstein 条件证明原始半代数蕴含，不再把 goal 边界矩形整体当作
drift 活动域。最终 W/q/epsbar 仍由同一个独立 validator 在原集合上复核，且
validator 不读取乘子。完整合同见
[`G8_SCALAR_MULTIPLIER_BERNSTEIN_CONTRACT.md`](G8_SCALAR_MULTIPLIER_BERNSTEIN_CONTRACT.md)。

### 9.4 G10 首片综合

为直接求取 `q<=0.10`，活动综合转为
`G10_CHEBYSHEV_POLYNOMIAL_MULTIPLIER_SPARSE_LP_V1`。G10 固定严格小于精确
`1/10` 的相邻 binary64 q，使用 total-degree Chebyshev W、局部非负 Bernstein
多项式乘子和稀疏 HiGHS LP 最大化共同 `rho`。有限 schedule 为
`W14/M2 -> W16/M2 -> W16/M4`；只有正且满足数值 LP 门禁的候选进入 V3，只有
V3 `validated + first_slice_ready` 才能发布 schema v2 certificate。完整合同见
[`G10_CHEBYSHEV_POLYNOMIAL_MULTIPLIER_CONTRACT.md`](G10_CHEBYSHEV_POLYNOMIAL_MULTIPLIER_CONTRACT.md)。

### 9.5 G10R 冗余消除路线

G10R 不覆盖 G10。它利用同-box Bernstein 锥的乘法与升阶封闭性，精确删除
每个 obligation 上不改变 `(W,rho,shape multipliers)` 投影可行域的 `g_x/g_y`
乘子；X0 disk 和 goal exterior shape multipliers 保持不变。G10R 使用独立
`solve-qsbc-g10r` 入口和 synthesis ID，schedule、q、epsbar、C2、G4 与 V3
发布门禁全部继承 G10。严格证明和冻结规模见
[`G10R_REDUNDANCY_REDUCED_LP_CONTRACT.md`](G10R_REDUNDANCY_REDUCED_LP_CONTRACT.md)。

### 9.6 G10E 有限高阶 fallback

G10E 不改变 G10 或 G10R。它继续使用 G10R 的同-box rectangle multiplier
精确投影等价消除，只把独立搜索空间冻结为
`W18/M4 -> W20/M4`，时间上限分别为 7200 s 和 10800 s。q、epsbar、C2、D1、
N1、系数界、partition、G4 义务和 V3 零容差发布门禁全部保持不变。首个通过
V3 的阶段立即停止；两阶段全部失败时只能报告 `not_ready`，不得自动继续 W22
或提高 multiplier 次数。独立入口为 `solve-qsbc-g10e`，严格合同和冻结规模见
[`G10E_EXTENDED_REDUCED_LP_CONTRACT.md`](G10E_EXTENDED_REDUCED_LP_CONTRACT.md)。

### 9.7 G10A V3 精确盒主动反馈 fallback

G10A 不覆盖 G10/G10R/G10E。它固定同一 global Chebyshev W、q、epsbar、
冻结 C2 和 V3，仅把 G9 diagnostic barrier 用作 warm start 与有限点行排序。
每四个 outer 运行一次 V3；只有 report/tree/hash 完整绑定的
`proved_violation` 或 `unresolved` 精确有理叶盒可以生成不含 rho 的硬
Bernstein 行。W16/W18/W20 的 outer/V3/time schedule 均有限，首个 V3
validated 停止，全部失败为 `not_ready`。独立入口为 `solve-qsbc-g10a`，
完整合同见
[`G10A_ACTIVE_BERNSTEIN_FEEDBACK_CONTRACT.md`](G10A_ACTIVE_BERNSTEIN_FEEDBACK_CONTRACT.md)。

### 9.8 G10A2 精确 Bernstein 预分离 fallback

G10A2 是不修改 G10A/G10E 的独立 W16 候选路线。它固定 `rho=0`，把哈希绑定的
G9 Chebyshev 系数作为投影中心，按 `min L_inf` 后 `min L1` 的两阶段目标产生四个
候选；固定 D3/D4/D5 目录只加入单个负 Bernstein 系数约束，不加入整盒约束块。
第四个候选必须完整扫描 42 个 family 的 D5 目录且无负系数，才允许恰好一次不变
V3。预分离器、LP、cut ledger 均不是形式证据；唯一发布条件仍是 schema-v2 V3
零容差重建、`validated`、`first_slice_ready=true` 和全部 subject/tree/hash 门禁。
任何失败均为 `not_ready`，不重试 V3，也不自动加次数、目录或预算。完整合同见
[`G10A2_EXACT_PRESEPARATION_CONTRACT.md`](G10A2_EXACT_PRESEPARATION_CONTRACT.md)。

### 9.9 G10A3 精确网格多点分离 fallback

G10A3 保持同一个 global W16、C2、q、epsbar 和不变 V3，只把候选生成改为冻结的
G17/G33/G65 exact-rational tensor grids。每个 family 按空间 strata 选择有限个最坏
点并加入 zero-rho point cuts，避免 G10A 的首个 DFS 叶盒长期遮蔽其他负区。四个
候选、八次 LP、点数、行数、内存和时间预算均有限；最终完整 G65 gate 仍不是形式
证据，只能允许恰好一次 V3。完整合同见
[`G10A3_EXACT_GRID_MULTIPOINT_CONTRACT.md`](G10A3_EXACT_GRID_MULTIPOINT_CONTRACT.md)。

### 9.10 G11P49 连续分片 barrier 与 roof witness fallback

G11P49 在固定 7×7 obstacle-aligned partition 上使用共享 Bernstein control net，
使 W 的 C0 连续性成为结构性质；148 个四象限 roof witnesses 对所有可达 target
pieces 上界 W，并通过精确半三角矩得到 drift 上界。该路线不由旧 V3 承担证明：
只有独立 `G5_EXACT_RATIONAL_PIECEWISE_ROOF_REPLAY_V3P_V1` 重建 partition、reach
catalog、roof 支配、积分 drift、controller 和全部 hash 后，才允许发布 schema-v3
证书。完整合同见
[`G11P49_C0_PIECEWISE_ROOF_CONTRACT.md`](G11P49_C0_PIECEWISE_ROOF_CONTRACT.md)。

## 10. Exact 与 Gazebo 的声明边界

决策 ID：`GZ1_EMPIRICAL_DIFFERENTIAL_TRACKER_V1`。

- encoder 只接受带明确 `world/odom` frame 关系的 odometry；缺失或跳变 fail
  closed；
- desired world velocity 转换为差速目标：

  ```text
  heading_des = atan2(u_y,u_x)
  v = ||u|| cos(wrap(heading_des-heading))
  omega = k_heading wrap(heading_des-heading)
  ```

- tracker sampling 为 `0.05 s`；
- actuator limits 在 Gazebo adapter 内显式记录；
- 当前没有形式 tracking-error bound。

因此 Gazebo 结果一律为 `empirical`。差速 tracker 中的 actuator saturation
不改变 exact certificate 的数学输入，也不允许声称继承 exact q-SBC bound。

## 11. 执行与产物规则

- 所有 Python、训练、求解、测试和仿真只在服务器运行；
- 本机只允许编辑、静态文本检查、Git 检查和原始归档复核；
- 每次服务器运行使用新的 timestamped source snapshot 和 result directory；
- 不依赖 `2026 experiment` 或原项目路径；
- 不导入旧 checkpoint、controller、certificate 或结果；
- `接手的原始文件` 只保存三份接手归档，不得新增、解压、改名或修改任何内容；
- controller、certificate、数据和 manifest 均用 SHA-256 绑定；
- solver output 与 independent validation 必须作为不同字段报告。

## 12. 验收

- [x] M1/S1/P1/D1/N1 已冻结；
- [x] K2/T2/V1/ST1 已冻结；
- [x] PB1/PA1/PS1 已冻结；
- [x] C1/CV1/C2V1 已冻结；
- [x] G4 theorem 已冻结；
- [x] G5/G6/G7/G8 solver/validator 已冻结；
- [x] exact/Gazebo claim boundary 已冻结；
- [x] server-only execution boundary 已冻结。

合同确认人：`YanShibo（推荐项统一授权）`
