# 证据与声明等级

安全相关命名不能替代证据。每个输出都必须说明它来自数学证明、显式模型误差界、
实验观察还是诊断计算。

## 四种等级

| 等级 | 最低证据 | 不代表 |
|---|---|---|
| `formal` | 完整定理、明确假设、精确兼容哈希、成功求解证据和独立验证 | 对未覆盖状态、场景或实现仍然安全 |
| `bounded_model_error` | 上述链路依赖一个显式且已验证的误差界 | learned/Gazebo 误差可忽略 |
| `empirical` | 可复现实验、完整 trace 与统计协议 | 形式化保证 |
| `diagnostic` | 定义清楚的计算量和适用域 | 任何安全声明 |

等级必须由制品与运行时兼容检查推导，不能只在 YAML 中手工填写一个更强标签。

## 证书制品的 formal 门槛

要具备 `formal` 声明资格，至少需要：

1. 已确认的 system、controller、scenario、noise 数学契约；
2. 状态顺序、单位、frame、采样周期和集合定义完全固定；
3. 定理中的所有证明义务、符号方向和概率语义明确；
4. solver 成功状态、版本、容差和原始证据可追溯；
5. 与生成路径独立的 validator 通过；
6. 运行时四个兼容哈希与 artifact 完全一致；
7. 所有假设在声明的适用域内成立；
8. 已知失效条件没有被触发。

[`artifacts/certificates/schema.yaml`](../../artifacts/certificates/schema.yaml)
记录这些字段，但“符合 schema”本身仍不是证明。

## 运行时 evaluation

公共 certificate contract 应保持小而稳定。特定 q-SBC evaluator 可以在
metadata 或专用类型中记录 `W(x)`、一步期望、残差、`q`、`epsilon` 和区域信息，
但这些字段不能强迫所有证书共享同一条定理。

运行时必须区分：

- `satisfied`：条件在数值容差内满足；
- `violated`：条件可评估但不满足；
- `not_applicable`：状态、动作或区域超出 artifact 适用域；
- `incompatible`：system/controller/scenario/noise 或 state schema 不匹配；
- `numerical_error`：非有限值、积分/期望失败或容差无法解释。

“不适用”和“数值错误”不能被记成满足，也不能悄悄降级为经验安全。

## evaluator、modifier 与 guard

- evaluator 只计算证书条件，不改变动作；
- modifier 提出候选动作并报告可行性、干预量和诊断；
- execution guard 决定不可行、超时和错误时是否拒绝命令、结束 episode 或使用
  经过论证的 fallback。

零动作不是通用安全 fallback。任何 fallback 都必须属于系统契约并有相应证据。

## 仿真的证据边界

exact simulator 可以检查公式实现、离散推进、期望计算、兼容门禁和可复现轨迹。
Gazebo 可以检查 ROS、传感器、时间、差速运动和集成行为。二者都不能单独生成
formal 声明。

若数学系统和 Gazebo rover 之间存在 tracking adapter，则 claim 只能覆盖数学
系统。只有 tracking error 被显式建模并验证后，才可能讨论
`bounded_model_error`。

## Learned RGB-D

learned RGB-D 当前固定为 `empirical`：

- CBF-like/CLF-like 是数据学习分数；
- latent safe set 未经独立几何验证；
- MDRNN 没有经验证的全局误差界；
- 有限步 Adam 动作修正不保证可行；
- 模型 manifest 的 `verified` 只覆盖身份和加载兼容性。

对应限制见
[`docs/baselines/LEARNED_RGBD.md`](../baselines/LEARNED_RGBD.md)。
