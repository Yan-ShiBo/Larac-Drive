# G10A3：精确有理网格多点分离合同

状态：**核心实现与统一 CLI 接入完成，待服务器正式运行**
版本：`G10A3-v1`
日期：`2026-08-01`

## 1. 边界与证明地位

G10A3 是 G10A/G10A2/G10E 旁边的独立有限 fallback。它不修改冻结的
M1/S1/P1/D1/N1、C2 controller、42 个 q-SBC family、15 个 controller
family、`q=nextafter(0.1,0)`、`epsbar=1e-6` 或 V3 validator。候选仍是一个
全局 W16 total-degree tensor-Chebyshev 多项式，153 个系数各限制在
`[-1000,1000]`。

精确网格、点切割、LP 状态和最终 G65 扫描都只是候选生成或拒绝证据，不能单独
支持论文中的形式化声明。唯一发布权属于对最终同一个 `W` 新执行的、subject-bound、
零容差 V3 完整验证。runtime 保持 `monitor_only`，certificate evaluator 不修改动作。

## 2. 为什么需要多点分离

G10A 的 V3 tree 对一个 family 在遇到首个 `unresolved` 叶后即可停止。实际运行中，
`W_nonnegative_on_X` 因而长期反馈左下角的 depth-20 未决盒，而没有遍历到右上角的
显式负区；例如 outer 12 在精确有理点 `(15/16,15/16)` 仍有 `W<0`。G10A3
不改变 V3，而是在调用 V3 前用固定、完整、无自适应扩张的有理网格寻找每个 family
的多个空间分离最坏点。

## 3. 固定网格与精确身份

对每个 family 的 V3 根盒

```text
B = [x_low,x_high] x [y_low,y_high]
```

定义

```text
G_n(B) = {
  (x_low + i(x_high-x_low)/(n-1),
   y_low + j(y_high-y_low)/(n-1))
  : i,j = 0,...,n-1
}.
```

固定 schedule 为：

| pass | grid | strata | 每 family 最多新点 |
|---|---:|---:|---:|
| G17_S8 | 17 x 17 | 4 x 4 | 8 |
| G33_S16 | 33 x 33 | 8 x 8 | 16 |
| G65_S32 | 65 x 65 | 8 x 8 | 32 |

节点、root box 和 domain classification 全部以精确有理数计算。网格 residual 使用
公共分母整数化的二维 tensor-Horner 求值；逐点结果与直接 `Fraction` evaluator 完全
相等，但不再对每个 monomial 重复做有理数乘方。
一个点作为零宽盒交给现有 exact disk/goal-exterior classifier；`outside` 跳过，
`inside` 与 `boundary` 都参与。每个 grid 的完整 point-set SHA-256 在候选之外冻结；
runtime 必须重算相同 family/grid/index/exact-point/domain 集合，计数相同但哈希不同
仍 fail-closed。目录不嵌入全部 235,326 个 point payload，只保存可流式复算的公式、
计数和集合哈希。family contract 同时绑定每个 family 经排序及 canonical `+0.0`
归一化后的完整 binary64 basis 系数，不能只绑定 root、次数和常数项。

## 4. 每 family 的空间分离规则

必须区分两个不能混同的精确 residual：

- `coefficient-space residual`：`exact(binary64 basis row) · exact(binary64 Chebyshev c)
  + exact(constant)`，它与 LP 决策变量和新增点行属于同一坐标空间；
- `artifact residual`：从已写出的 binary64 monomial barrier artifact 独立重构的形式化
  residual，它包含 Chebyshev 到 monomial 的 float64 累加舍入。

前三轮只用 coefficient-space residual 排名和选点；候选 residual 小于
`exact_binary64(1e-8)`，且最终 float64 行对完整 308 维候选实际重放满足
`A @ x - b > 1e-9` 的点才有资格生成 hard cut。不能用
`float(exact residual)` 代替实际矩阵点积。普通索引的 stratum 为

```text
floor(index * strata_per_axis / (nodes_per_axis - 1))
```

最后一个 endpoint 固定属于最后一个 stratum。每个 stratum 先按精确 residual、
`i`、`j` 选最坏的尚未加入点，再从这些 winner 中取本轮每 family 的固定上限。
相同 family、相同精确点、相同 exact row 和 margin 在嵌套网格间只允许一次；grid
身份只进入 selection event，不使同一物理点变成新的 constraint definition。

三轮最多生成

```text
42 * (8 + 16 + 32) = 2352
```

个单行 point cuts。每行保持原始 residual 方向：

```text
f_j(W; xi_exact) >= 1e-8.
```

`rho/t/z` 列必须是 canonical `+0.0`。definition hash 绑定 family contract、精确点、
精确 artifact row、margin 和 zero-rho 语义；selection-event hash 另外绑定候选 barrier、
完整 308 维 binary64 求解向量的 SHA-256、coefficient-space residual、artifact residual、
`artifact-coefficient` 精确差值、实际 float64 violation 的 hex、pass 与 grid point
identity。ledger validator 由期望 grid catalog、冻结 families 和对应 candidate subject
重算 exact row/constant/RHS、两种 residual、转换差值以及原始完整 308 维向量上的
`A @ x - b`。禁止重建后缀为零的伪候选；任一位或哈希不一致均 fail-closed。

## 5. 候选 LP 与最终网格门禁

求解布局和 G10A2 的稳定 projection 相同：

```text
[c_0,...,c_152, rho, t, z_0,...,z_152]
```

`rho` 固定为 canonical `+0.0`。每个候选使用两阶段目标：先最小化相对 hash-bound
G9 seed 的 Chebyshev 系数 `Linf` 距离，再在
`t <= nextafter(t_star,+inf)+1e-9` 下最小化 `L1` 距离。禁止 pure-L1 first。
primary master matrix 与带临时 lexicographic 行的 secondary matrix 都在求解前按
G10A3 自身的 row/nonzero/RSS 上限和 G10A3 标签采样。

固定生成四个候选、执行八次 LP：

```text
G9 anchors -> C0
C0 + G17 cuts -> C1
C1 + G33 cuts -> C2
C2 + G65 cuts -> C3
```

对 C3 再完整扫描一次 G65，但不再加 cut 或重求。必须访问
`42*65^2=177450` 个 family-point identity；该最终门禁只读取 binary64 monomial barrier
artifact 的精确 residual。任一 active point 的 artifact residual `<0` 即直接
`not_ready`，不调用 V3。coefficient-space residual 仍记录为诊断，但无权替代 artifact
门禁。通过该门禁也不是证明，只允许进入唯一一次 V3。

## 6. 固定资源上限

- aggregate LP wall time：1800 s；
- aggregate exact-grid wall time：3600 s；
- complete stage wall time（包含 V3 与发布）：10800 s；
- process `VmHWM`：1 GiB，缺失测量即失败；
- catalog generation point yields：235,326；
- candidate scan point visits：412,776；
- total grid point yields：648,102；
- selected hard point rows：2352；
- master rows：3006；临时 lexicographic rows：3007；
- CSR nonzeros：400,000；
- decision variables：308；
- full V3 calls：1。

任一 deadline、RSS、row、nonzero、visit、hash、ledger、solver 或 late-return 失败均
fail-closed。禁止自动增加 grid、点数、轮次、次数、degree、time budget 或第二次 V3。

## 7. 唯一发布门禁

写出 schema-v2 certificate 前必须同时满足：

```text
complete final G65 exact grid gate passed
grid catalog / visit count / point-set hash / cut ledger passed
all stage deadline and VmHWM gates passed
exactly one unchanged V3 call
V3 schema_version = 2
V3 validator_id = G5_EXACT_RATIONAL_BERNSTEIN_REPLAY_V3
V3 status = validated
V3 global_polynomial_status = validated
V3 first_slice_ready = true
V3 controller_contract_status = validated
V3 geometry_contract_status = validated
V3 theorem_scalar_gate = true
V3 formal_acceptance_tolerance = 0
all barrier/controller/q/epsbar/model/report/tree bindings passed
unchanged V3 release gate passed
```

发布采用 `certificate.pending.json` 到 `certificate.json` 的同目录原子提升，并在写前、
pending 写后、rename 后和 ready manifest 前重复 stage deadline 与 peak RSS 门禁。
预期 certificate 必须先在内存中构造并保持为可信对象，再写 pending；禁止把首次磁盘
load 的结果反过来当作 expected certificate。即使 pending 在首次 load 前被替换为另一个
内部自哈希合法的 schema-v2 certificate，也必须因 trusted barrier/q/epsbar/controller/V3
validation 交叉绑定不一致而拒绝。
最终一次 ledger save 返回的内存 payload、其 file/content SHA-256 与累计 row count 也是
可信状态；第一次及随后每次磁盘回读都只能与该状态做完整对象相等检查，不能用回读结果
覆盖 expected ledger。即使 ledger 在首次回读前被替换为 `rows=[]`、`row_count=0` 且
content hash 内部自洽的合法空账本，也必须因 payload、file/content hash 和累计行数绑定
不一致而拒绝，且不得进入 V3。
紧邻 rename 前必须从磁盘重新读取并重放 pending certificate、V3 report、全部 V3 tree、
ledger 引用的每个 `candidate_*/numeric_candidate.json`、point-cut ledger、exact-grid catalog
和 final G65 report。candidate 必须与运行中可信 manifest 记录的 file/content/barrier/full-
vector hash 一致；ledger replay subject 只能由这些磁盘 candidate 重建，不能继续使用内存
对象。certificate solver metadata 还必须绑定 ledger 的 file SHA-256 与 content SHA-256。
随后重新执行 subject-binding、V3 release 及 G10A3 publication gate；rename 后还要重新
载入最终 certificate 并与预期主体逐字段及 content hash 相等。任何异常只清理 G10A3
自己的 pending/final 文件，并返回 `not_ready`。

完整配置位于 `configs/certificates/g10a3_qsbc.yaml`。正式研究结果必须在服务器新空
输出目录运行并保存 source snapshot、输入哈希、命令、stdout/stderr、资源日志、所有
grid report、point-cut ledger、candidate、V3 report/tree 与最终 `SHA256SUMS`。本地
import、静态检查或小测试不构成研究结论。
