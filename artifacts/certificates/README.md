# Certificate artifacts

本目录定义一等公民的证书制品。当前只有
[`schema.yaml`](schema.yaml)，没有现有 q-SBC、CBF 或其他证书，也没有伪造的
系数、求解状态或验证记录。

未来每个证书制品必须：

1. 绑定 system、controller、scenario 和 noise 配置的内容哈希；
2. 固定状态顺序、单位、坐标系、离散时间和数值表示；
3. 保存系数或绑定外部系数文件及其 SHA-256；
4. 保存求解器、终止状态、数值容差和证据位置；
5. 由与生成路径独立的 validator 复核；
6. 完整列出假设、适用域和已知失效条件；
7. 由兼容性和验证结果计算声明资格，不能手工把文件标成 `formal`。

exact 仿真和 Gazebo 都是运行证据，不代替上述证明与验证链。
