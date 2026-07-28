# Learned RGB-D baseline

此目录只保存 inherited learned RGB-D 方法及其 depth PPO 参考策略配置。它们是
默认禁用的经验基线，不属于数学证书主线。

- `method.yaml`：VAE、MDRNN、CBF-like/CLF-like 分数和动作修正参数；
- `reference_controller.yaml`：尚未实现的 depth PPO 参考 controller 契约。

五个模型槽位都仍为 `missing`，详见
[`artifacts/models/manifest.yaml`](../../../artifacts/models/manifest.yaml)。
