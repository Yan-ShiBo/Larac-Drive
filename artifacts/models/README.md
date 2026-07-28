# Learned model artifacts

[`manifest.yaml`](manifest.yaml) 是 learned RGB-D 五个外部模型槽位的权威清单，
[`schema.yaml`](schema.yaml) 定义身份、来源、完整性、兼容性和状态迁移。

仓库不保存模型权重。文件从 `CBF_ROVER_ARTIFACT_ROOT` 解析，并在服务器校验
SHA-256、字节数、加载器及最小输入输出契约。当前所有槽位均为 `missing`。
四个 learned modifier 槽位标为 manifest `required`；`reference_policy` 对 modifier
本身是可选槽，但对 paired depth-PPO 实验仍是配置级必需项。

`verified` 只证明清单指向了预期文件，且目标加载器能够按声明契约读取它。该状态
不证明控制器安全，不会把 CBF-like/CLF-like 学习分数升级为数学证书。
