# 障碍证书方法

## 方法分级

每个方法必须明确声明证据等级，名称本身不能替代证据：

| 等级 | 含义 |
|---|---|
| `formal` | 假设、动力学、安全集和证明义务完整，并有可审查证明 |
| `bounded_model_error` | 保证依赖显式且已验证的模型误差界 |
| `empirical` | 结论仅来自给定数据与实验分布 |
| `diagnostic` | 只输出诊断量，不作安全声明 |

learned RGB-D 当前为 `empirical`。其 CBF-like/CLF-like 输出是学习分数，不是
形式化控制障碍函数或控制 Lyapunov 函数。

## 稳定接口

新方法放入 `src/cbf_rover/methods/<method_id>/`，通过 `core` 中的稳定协议提供：

1. 输入状态或观测的结构与单位；
2. 控制量、扰动和动作边界；
3. 安全集、目标集和符号方向；
4. 证书或诊断评估结果；
5. reference action 到 final action 的过滤结果；
6. episode 开始/结束时的状态重置；
7. 可行性、违规量和失败原因；
8. 声明等级、数学假设和已知失效条件。

方法不得直接发布 ROS 消息。ROS2 适配层负责时间同步、消息转换和命令发布。

## 新方法接入清单

- 在 `configs/methods/` 新增唯一方法配置；
- 在 `docs/methods/` 写明数学定义和证据等级；
- 不复制场景 seed、运行环境或记录字段；
- 为所有数值容差注明单位、尺度和采样周期；
- 提供无模型或缺制品时的 fail-closed 行为；
- 与相同场景、初始状态和 seed 的 reference policy 成对比较；
- 保存逐步 trace、episode 汇总、配置快照和制品清单快照；
- 分开报告安全指标、任务性能和干预强度。

任何尚未达到形式证明条件的方法必须使用 `learned_*`、`approximate_*` 或
`diagnostic_*` 命名，不得标为 `formal_certificate`。
