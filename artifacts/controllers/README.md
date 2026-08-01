# Frozen controller artifacts

本目录只保存 controller 制品协议，不保存运行生成的系数。生产 controller 位于
服务器运行根目录，并由 `schema.yaml`、构造来源 SHA-256、内容 SHA-256 和对应
validation report 共同约束。

统一入口 `load_frozen_controller()` 同时支持历史 PB1 全局多项式 schema v1 和
当前 C2 分段多项式 schema v2。只有哈希校验、完整重建及对应 CV1/C2V1 状态均
为 `validated` 的文件，才能成为 q-SBC 的只读输入。

C2 在共享 x 边界使用“运行时按顺序首个匹配、形式验证覆盖相邻两个闭包”的固定
语义。证书合成不得修改 piece、系数、边界或运行时动作。
