# Certificates

这里保存运行时 certificate evaluator 的选择与参数，并引用
[`artifacts/certificates/`](../../artifacts/certificates/) 中经过独立验证的证书
制品。

当前已有 G4/G10/G10R/G10E/G10A/G10A2/G10A3/G11P49 综合配置，但仓库内仍没有
可作为活动发布结果的证书系数。全局多项式路线只有独立 V3 零容差复核及全部哈希
门禁通过的 schema-v2 制品才能成为活动证书；P49 路线则必须由独立 V3P 发布
schema-v3 制品。求解器状态、LP 乘子、网格或数值裕量本身都不是形式证明。

G10A2 只生成候选：固定 `rho=0`，使用 D3/D4/D5 精确 Bernstein 系数预分离，
最终完整 D5 门禁通过后最多调用一次不变 V3。预分离报告与 cut ledger 均不被 V3
读取，也不能单独发布证书。配置见 [`g10a2_qsbc.yaml`](g10a2_qsbc.yaml)。

G10A3 使用冻结 G17/G33/G65 精确网格和多点分离来避免单叶 DFS 反馈偏置；完整
G65 gate 仍只是进入唯一 V3 的前置诊断。配置见
[`g10a3_qsbc.yaml`](g10a3_qsbc.yaml)。G11P49 使用 C0 分片 barrier 与 quadrant
roof witness，并由独立 V3P 验证；配置见 [`g11p49_qsbc.yaml`](g11p49_qsbc.yaml)。
