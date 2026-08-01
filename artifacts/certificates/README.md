# Certificate artifacts

本目录定义一等公民的证书制品。当前只有
[`schema.yaml`](schema.yaml)，没有现有 q-SBC、CBF 或其他证书，也没有伪造的
系数、求解状态或验证记录。

证书读取器支持三种明确分开的格式：schema 1 是只读兼容的全局稀疏多项式
旧格式；schema 2 是当前活动的全局稀疏多项式格式，使用 V3 exact-rational
Bernstein replay；schema 3 只用于 G11/P49 分片 Bernstein barrier、148 个 roof
witness 和独立 V3P replay。支持 schema 3 不代表已经生成或发布了 schema 3 证书。

每个 G4 q-SBC 证书制品必须：

1. 绑定冻结 controller 的内容哈希，并使用活动 M1/P1/N1/G4 配置；
2. 固定状态顺序、单位、坐标系、离散时间和数值表示；
3. 保存系数或绑定外部系数文件及其 SHA-256；
4. 保存求解器、终止状态、数值容差和证据位置；
5. 由不读取 solver status、LP/SOS multiplier 或 Gram matrix 的相应独立验证器
   以零容差重建复核：schema 1/2 使用 V3，schema 3 使用 V3P；
6. 完整列出假设、适用域和已知失效条件；
7. 由兼容性和验证结果计算声明资格，不能手工把文件标成 `formal`。
8. validation report 必须绑定 barrier/controller/q/epsbar，并以 SHA-256 绑定全部
   证明树；schema 2/3 的派生概率下界和停止时间上界必须同时保存精确分数并向外舍入；
9. schema 3 还必须交叉绑定 P49 geometry、roof witness、formal model、V3P report
   及全部 1126 个唯一证明树，并在原子发布前从磁盘完整复核一次。

exact 仿真和 Gazebo 都是运行证据，不代替上述证明与验证链。
当前目录没有已发布的首片 q-SBC 证书；G11/P49 仍须服务器数值求解与 V3P 全部
通过后，才能产生 `certificate.json` 并声明 `ready`。
