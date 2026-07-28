# 模型制品清单

仓库不保存模型权重，也没有活动默认模型。当前权威清单是
[`manifests/models.yaml`](manifests/models.yaml)，其中五个槽位均为
`status: missing`。这是一种明确的初始状态，不表示模型存在、可加载或已验证。

模型文件必须保存在仓库外，由 `CBF_ROVER_ARTIFACT_ROOT` 指定根目录。清单中的
`uri` 只能在该根目录内解析；服务器加载前必须校验 SHA-256、字节数、来源和
兼容信息。完整字段与状态迁移规则见
[`manifests/schema.yaml`](manifests/schema.yaml)。

只有同时满足以下条件的制品才能从 `missing` 或 `staged` 改为 `verified`：

1. 文件由服务器重新训练产生，或从可信外部存储显式注入；
2. 清单记录不可变 URI、SHA-256、字节数和训练运行 ID；
3. 模型架构、框架版本、输入输出约定与加载器均已登记；
4. 在目标服务器完成实际加载与最小推理验证；
5. 清单变更经过人工审查。

`verified` 只证明制品身份和加载兼容性，不证明控制安全性。学习型 RGB-D 方法
始终按 `claim_level: empirical` 报告，除非未来另有独立的形式化证明。
