# G10A：V3 精确盒反馈的主动 Bernstein LP 合同

状态：**IMPLEMENTED，PENDING SERVER GATES AND SOLVE**

版本：`G10A-v1`
日期：`2026-08-01`

## 1. 结论与边界

G10A 是 G10/G10R 旁边的独立 fallback，不修改二者。它保持同一套
M1/S1/P1/D1/N1、冻结 C2、G4 四项 q-SBC 义务、
`q=nextafter(0.1, 0)`、`epsbar=1e-6` 和 V3 exact-rational validator。

G10A 的 LP 只是候选生成器。G9、主动行、HiGHS 状态、共同松弛量
`rho` 和数值 slack 都不构成证明。唯一发布门禁仍是 V3 对最终单个全局
多项式 `W` 的独立重建与零容差证明。

## 2. 审计所得的数据流

现有 G9/DX2 的身份是
`DX2_SAMPLED_QSBC_EXPRESSIVITY_LP_V1`。其网格即使包含形式分区断点，
仍只是有限采样；其 barrier 明确标记为 diagnostic-only，不能成为证书。
G10A 只读取一个内容哈希绑定的 G9 JSON，并允许它做两件事：

1. 为 HiGHS 提供可选的 Chebyshev 数值 warm start；
2. 对 V3 根盒的有限 dyadic 子盒中点进行 residual 排序。

服务器现有 W14 probe 的历史 `report_content_sha256` 与当前规范重算值不一致，
因此不能把该旧文件直接当作已通过内容哈希门禁的输入。G10A 要求一个当前规范
自哈希有效的 `g10a_g9_seed_bundle`。bundle 只允许引用同目录 sibling probe，
绑定其 raw file SHA-256，并同时记录旧声明哈希、当前重算哈希和两者不相等；同时
逐字段复制并核对 barrier、q、epsbar、degree、basis 与 Chebyshev 系数。旧 probe
仍保持 diagnostic-only，bundle 也不会提升其证明地位。直接传入该非规范旧 probe
必须 fail-closed。

每个 V3 q-SBC family 先激活一个最差 G9 中点，之后每个 outer 最多加入
12 个同类诊断中点。中点行形如

```text
f_j(W; x_mid) >= rho
```

这些行不覆盖连续集合，也不会被 V3 读取。

V3 report schema 为 2。当前冻结模型恰有 42 个 q-SBC family 和 15 个
controller family。每个 family 的独立 tree 保存 `box_exact`、
`domain_class`、精确上下界和 disposition。G10A 只接受 q-SBC tree 中：

```text
proved_violation
unresolved
```

两类 terminal leaf。`accepted_by_exact_lower_bound` 和
`pruned_outside_domain` 不生成反馈行。controller、geometry 或 theorem
scalar gate 失败不能通过改变 `W` 修复，阶段立即 fail-closed。

## 3. V3 反馈约束

设 terminal leaf 给出的精确有理盒为 `B`，对应原始义务 residual 为
`f_j(W, xi) >= 0`。G10A 使用 `box_exact` 的端点构造该 residual 在 `B`
上的 tensor-Bernstein 系数：

```text
beta_jk(W; B) >= 1e-8    for every coefficient k
```

该行块不含 `rho`，所以会在 `1e-9` LP feasibility tolerance 之外排除具有负
Bernstein lower bound 的当前候选；不会仅通过把 `rho` 变负而保留同一候选。
`1e-8` 只是候选 LP 的数值
tightening，不改变 q、epsbar 或 V3 的零容差义务。

LP 保留 `box_exact` 的规范有理数字符串与端点 float64 hex；构造候选行时才把
端点转换为 binary64。这里的“exact box feedback”指反馈几何和 provenance
来自 V3 精确树，不表示候选 LP 改用有理数求解。最终形式结论仍只来自 V3 的
全程 exact-rational 重建。

对 `domain_class=inside`，整盒 Bernstein 非负是原始义务的充分条件。
对 `domain_class=boundary`，G10A 也要求整个 leaf box 非负。这会约束盒内
少量非活动点，因此比原问题保守；但 V3 的 boundary leaf 已达到其精确
subdivision budget，该保守范围很小。这个选择可能导致 false negative，
不会导致 false positive。最终是否成立仍由 V3 在真实半代数域上判断。

每个反馈前必须同时验证：

- report canonical content SHA-256；
- validator/schema/零容差身份；
- barrier payload、degree、term count；
- 冻结 controller content SHA-256；
- q/epsbar 的 exact binary64 和 hex；
- formal model payload hash；
- 42 个 q-SBC family 的精确集合；
- 冻结 C2 导出的 15 个 controller family 的精确名称集合；
- formal model payload 与当前 M1/S1/P1/D1/N1 canonical payload 完全相等；
- 每个 report summary 与独立 tree 文件的 SHA-256 和无 leaves 摘要一致。

任一绑定失败都不得生成反馈行。

## 4. 固定有限 schedule

| stage | degree(W) | max outer | max V3 | aggregate LP time |
|---|---:|---:|---:|---:|
| W16_ACTIVE | 16 | 12 | 3 | 3600 s |
| W18_ACTIVE | 18 | 16 | 4 | 7200 s |
| W20_ACTIVE | 20 | 20 | 5 | 10800 s |

每四个 outer 是一个 V3 checkpoint，因此上表的 V3 次数上限分别为
3/4/5。首个通过完整 V3 release gate 的候选立即停止。所有阶段结束仍无
通过者时，唯一允许的状态是 `not_ready`；不自动升阶或扩大 schedule。

G10A 不用 `rho>0` 阻止 V3 checkpoint。主动行既不完整，又可能因 boundary
整盒策略而保守，负 rho 不能推出原始义务失败；反过来，正 rho 也不能推出原始
义务成立。因此只要有有限 binary64 candidate 就交给 V3，形式状态完全由 V3
决定。

## 5. 求解器与降级路径

首选后端为 public `highspy`：首次传入 CSR 模型，随后通过 `addRows`
增量加入诊断行和 V3 盒行，并让 simplex 保留现有 basis；首次增量后关闭
presolve，避免 presolve 映射破坏 warm basis。

`addRows` 的 `starts` 严格按 public API 传入每个新增行的一个起点（长度为
`num_new_row`，末端由 `num_new_nz` 给出）。HiGHS 的 run clock 会跨重复
`run()` 累计，因此每次调用把 absolute `time_limit` 设为
`getRunTime() + 本次委派的剩余 wall budget`。外层单次 solve 使用 monotonic
deadline：若 highspy 已耗时后 API 失败，只把 deadline 的真实剩余量交给 SciPy
fallback；两次尝试及包装开销全部计入同一个 aggregate LP 时间。任何后端在
deadline 后才返回的 candidate 都被丢弃，不得进入 V3。

若 import、model、option 或 `addRows` API 任一步异常，阶段永久切换到
SciPy/HiGHS 对当前完整 CSR 的 deterministic rebuild。不会在两个后端间
来回切换。后端、basis valid 状态、迭代数、用时和 fallback error 全部写入
manifest。两条路径产生的都只是 numeric candidate。

## 6. 发布门禁

只有以下条件全部成立才写出 schema-v2 certificate：

```text
V3 status = validated
V3 first_slice_ready = true
V3 acceptance_tolerance = 0
exact_binary64(q) <= 1/10
exact_binary64(epsbar) > 0
barrier/controller/report/tree/model bindings all pass
```

运行入口（必须在服务器使用新的空输出目录）：

```text
python -m cbf_rover.entrypoints.research solve-qsbc-g10a \
  --controller <frozen_controller.json> \
  --g9-seed <g10a_g9_seed_bundle.json> \
  --output-dir <new-empty-directory>
```

本地 import、静态检查或小矩阵测试不构成研究结果。正式结果必须保存 source
snapshot、输入哈希、命令、stdout/stderr、资源日志、每次 V3 report/tree、
active-row ledger、candidate 和最终 SHA256SUMS。
