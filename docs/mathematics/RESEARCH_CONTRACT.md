# 首个 q-SBC 研究契约

状态：**等待用户确认，尚未形成活动数学实现。**

本文件是首个 q-SBC vertical slice 的输入契约。它必须在新增
`configs/systems/*.yaml`、`configs/controllers/*.yaml`、活动 certificate
配置或证书 artifact 之前完成。下面的候选内容用于聚焦讨论，不是已接受定理。

## 1. 研究问题

请确认首个闭环要回答的唯一问题：

> 在指定离散随机系统、固定 nominal controller、初始集、unsafe set 和有限时域
> 下，哪一条 q-SBC 定理给出什么精确概率结论？

需要填写：

| 项目 | 用户确认值 |
|---|---|
| 论文/定理来源或自定义 theorem ID | `TO_CONFIRM` |
| 安全事件的精确定义 | `TO_CONFIRM` |
| 时间范围：一步、有限时域或无限时域 | `TO_CONFIRM` |
| 概率结论中 `q` 的定义 | `TO_CONFIRM` |
| `epsilon` 或其他松弛量的定义 | `TO_CONFIRM` |
| 最终允许使用的声明句 | `TO_CONFIRM` |

禁止从变量名猜测结论。特别是不能默认把 `1-W(x)` 当作即时成功概率，也不能在
未确认定理时把 `1-q` 写进报告。

## 2. 候选最小系统

建议从二维离散随机单积分器开始，仅作为待确认候选：

```text
x[k+1] = x[k] + dt * u[k] + w[k]
x = [x_m, y_m]
u = [u_x_mps, u_y_mps]
```

请确认或替换：

| 项目 | 候选值 | 用户确认 |
|---|---|---|
| 状态顺序 | `[x_m, y_m]` | `TO_CONFIRM` |
| 控制顺序 | `[u_x_mps, u_y_mps]` | `TO_CONFIRM` |
| frame | `world` | `TO_CONFIRM` |
| 采样周期 `dt` | 未指定 | `TO_CONFIRM` |
| 控制边界 | 未指定 | `TO_CONFIRM` |
| 连续/离散建模关系 | 直接离散模型 | `TO_CONFIRM` |
| 终止时域 | 未指定 | `TO_CONFIRM` |

如果首个系统应为 unicycle、差速车或其他模型，应在此处改动，而不是让 Gazebo
adapter 隐式转换。

## 3. 集合

候选场景来自
[`configs/scenarios/static_obstacles.yaml`](../../configs/scenarios/static_obstacles.yaml)，
但场景几何不自动成为定理集合。请确认：

| 集合 | 必须明确的数学表示 | 用户确认 |
|---|---|---|
| domain `X` | 闭/开边界、二维几何、是否含 heading | `TO_CONFIRM` |
| 初始集 `X0` | 不等式或多项式表示 | `TO_CONFIRM` |
| unsafe set `Xu` | 三个 inflated obstacle 的并集如何编码 | `TO_CONFIRM` |
| goal set `Xg` | 是否参与 q-SBC 定理 | `TO_CONFIRM` |
| 边界外状态 | 终止、违规或不适用 | `TO_CONFIRM` |

robot footprint 与 inflation 的建模方式也必须确认。场景当前使用保守圆 footprint，
这只是显式设计值，不是已经证明正确的集合近似。

## 4. 噪声与期望

请填写：

| 项目 | 用户确认值 |
|---|---|
| `w[k]` 的分布或允许集合 | `TO_CONFIRM` |
| 各维相关性和时间独立性 | `TO_CONFIRM` |
| 均值、协方差或高阶矩 | `TO_CONFIRM` |
| 有界/无界支持 | `TO_CONFIRM` |
| 一步期望的计算方式 | `TO_CONFIRM` |
| 解析矩、数值积分或采样误差处理 | `TO_CONFIRM` |
| noise config 的内容哈希规则 | `TO_CONFIRM` |

若期望由 Monte Carlo 近似，就必须把采样误差和置信度纳入 theorem/claim；不能把
抽样值当成精确期望。

## 5. Controller

首个证书应绑定一个明确的 controller：

| 项目 | 用户确认值 |
|---|---|
| controller ID 与公式 | `TO_CONFIRM` |
| 固定 controller 还是同时综合 | `TO_CONFIRM` |
| 动作饱和发生在公式前还是后 | `TO_CONFIRM` |
| 随机 controller 是否允许 | `TO_CONFIRM` |
| controller config hash | `TO_CONFIRM` |

如果证书与控制器共同综合，artifact 必须同时记录两者表示及 solver 证据。

## 6. q-SBC 公式

此处必须由用户粘贴或确认完整公式。至少包含：

1. `W(x)` 的定义域、值域和符号方向；
2. 初始集上的条件；
3. unsafe set 上的条件；
4. domain 内的一步期望/漂移条件；
5. `q`、`epsilon`、时域和最终概率结论的关系；
6. controller 与噪声进入条件的方式；
7. 所有严格/非严格不等式和数值容差；
8. 不满足条件时的声明边界。

```text
THEOREM_AND_OBLIGATIONS = TO_CONFIRM_BY_USER
```

在该占位被替换并审查前：

- 不实现专用 `SBCEvaluation` 公式；
- 不生成或登记证书系数；
- 不把 solver 输出标成 q-SBC；
- 不在图、日志或论文结果中输出安全概率。

## 7. 求解与独立验证

请确认：

| 项目 | 用户确认值 |
|---|---|
| 多项式次数/基函数 | `TO_CONFIRM` |
| SOS/SDP 或其他求解形式 | `TO_CONFIRM` |
| solver 与版本 | `TO_CONFIRM` |
| 数值容差和缩放 | `TO_CONFIRM` |
| infeasible/unknown 的处理 | `TO_CONFIRM` |
| independent validator | `TO_CONFIRM` |
| validator 不复用哪些生成结果 | `TO_CONFIRM` |

建议优先接入已有研究求解与验证代码，由其输出
[`CertificateArtifact`](../../artifacts/certificates/schema.yaml)，不要在 runtime
中重写求解器。

## 8. Exact 与 Gazebo 映射

exact backend 应直接实现本契约的状态推进。Gazebo 差速 rover 则需要确认：

| 映射 | 用户确认值 |
|---|---|
| odometry 到数学状态的 encoder | `TO_CONFIRM` |
| `[u_x,u_y]` 到 `(v,omega)` 的 tracker | `TO_CONFIRM` |
| tracker sampling 与 delay | `TO_CONFIRM` |
| tracking error 界 | `TO_CONFIRM_OR_NONE` |
| 没有误差界时的 claim level | 建议 `empirical` | `TO_CONFIRM` |

Gazebo 几何和 topic 来自 `ScenarioSpec`，但运行适配不能修改数学假设。

## 9. 验收用例

研究契约确认后，首个实现至少需要：

- 3–5 个手算状态的一步推进、`W`、期望和残差 golden cases；
- 集合边界内外及三个 unsafe component 的测试；
- config/hash/state-order 不匹配时 fail closed；
- `not_applicable`、`violated`、`numerical_error` 分开；
- 固定 seed 的 exact trajectory 可重现；
- evaluator 不修改 nominal action；
- 不可行 modifier 在 environment step 前停止；
- trace 保存完整输入、artifact identity 和 evaluation；
- 不导入 Torch、ROS 或 Gazebo 的纯数学测试路径。

这些测试留到服务器执行。本文件确认本身不代表测试通过。

## 10. 用户签署

只有以下项目全部完成后，才能把状态改为 `confirmed`：

- [ ] 系统公式、状态顺序、单位、frame 和 `dt` 已确认；
- [ ] controller 已确认；
- [ ] domain、初始、unsafe、goal 集合已确认；
- [ ] noise 与期望计算已确认；
- [ ] q-SBC 定理和概率结论逐字确认；
- [ ] solver、容差和独立 validator 已确认；
- [ ] exact/Gazebo 的 claim 边界已确认；
- [ ] 允许的论文/报告措辞已确认。

确认人：`TO_CONFIRM`

确认日期：`TO_CONFIRM`

研究契约版本：`draft-1`
