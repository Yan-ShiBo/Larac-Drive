# Learned RGB-D 经验基线

## 定位

learned RGB-D 是从旧工程抽取并规范化的经验型对照方法，用于研究学习表征、
潜在动力学和动作过滤的组合行为。保留该方法不代表认可其安全保证。

方法链为：

```text
RGB-D -> VAE latent -> MDRNN next latent -> CBF-like / CLF-like scores
depth -> reference policy action ---------------------------> action filter
```

权威参数位于 `configs/methods/learned_rgbd.yaml`，参考策略约定位于
`configs/policies/depth_ppo.yaml`。

## 当前不可执行

本次重构删除了五个继承权重：

- reference policy；
- RGB-D VAE；
- RGB-D MDRNN；
- learned CBF-like 网络；
- learned CLF-like 网络。

`artifacts/manifests/models.yaml` 将所有槽位明确标为 `missing`，方法配置也保持
`enabled: false`。不得用随机初始化权重冒充基线结果。

恢复方法只有两种：

1. 在服务器按可追溯训练流程重新训练；
2. 从可信外部制品存储注入，补齐来源、哈希和兼容信息并完成服务器加载验证。

原始归档中的旧权重只用于来源追溯，不得直接变成活动默认模型。

## 制品加载边界

推理不接受四个任意文件路径。必须从版本化 manifest 和仓库外 artifact root
构造经过验证的 checkpoint set：

```python
from cbf_rover.methods.learned_rgbd import (
    LearnedRGBDCheckpointSet,
    LearnedRGBDSafetyConfig,
    LearnedRGBDWorldSafetyFilter,
)

checkpoints = LearnedRGBDCheckpointSet.from_manifest(
    manifest_path="artifacts/manifests/models.yaml",
    artifact_root="/absolute/server/artifacts",
)
method = LearnedRGBDWorldSafetyFilter(
    checkpoints=checkpoints,
    config=LearnedRGBDSafetyConfig(),
    device="cuda",
)
```

manifest 必须为 `artifact_set.status: ready`，四个方法槽位必须为
`status: verified`；文件 URI 必须留在 artifact root 内，且字节数和 SHA-256
逐一匹配。Torch 只以 `weights_only=True` 加载；旧版本不支持时明确失败。

## 经验性边界

- CBF-like 分数“更低更安全”是继承的工程约定，不是通用 CBF 符号。
- VAE 表征误差、MDRNN 预测误差和有限数据分布都会影响分数。
- 使用高斯混合均值预测可能掩盖多模态尾部风险。
- 有限步 Adam 动作修正不保证可行，也不等价于解析 QP。
- latent 中的安全集没有经过独立几何或可达性验证。
- 当前没有模型误差界、前向不变性证明或独立验证器。

因此，只能报告“在指定场景、制品、配置和 seed 下观察到的经验结果”。不得使用
“已证明安全”“形式化验证通过”或等价措辞。

## Episode 状态

MDRNN 隐状态必须在每个 episode 开始时重置。记录中必须包含 episode ID、seed、
逐步 reference/final action、分数、违规量、干预量、碰撞、障碍距离和终止原因。
缺少明确 reset 事件的运行不得纳入对比结果。

环境成功执行动作后，runner 通过 `update_after_step` 用实际执行动作推进一次
MDRNN 隐状态；环境若裁剪命令，必须在 transition 中显式回传
`executed_action`，不能让方法使用请求动作代替实际动作。

## 验证状态

本次没有在本机加载模型、运行推理、执行测试、构建 ROS2 或启动 Gazebo。上述
内容是架构与声明边界，不是运行通过记录。
