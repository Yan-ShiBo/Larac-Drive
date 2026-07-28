# ADR-0001：采用 math-first 五层架构

- 状态：Accepted
- 日期：2026-07-28
- 决策范围：Python 研究核心、配置、制品和 ROS2 边界

## 背景

前一阶段的目录边界并不混乱，但活动能力主要围绕 learned RGB-D、训练和 latent
action filter。用户的长期工作重心是障碍证书、数学更新与仿真，没有实车。继续
在 learned 路径上增加节点，会让证书数学、实验编排和 ROS 生命周期耦合。

另一个问题是证书、action modifier 与模型 manifest 的证据语义混在一起。模型
可加载、仿真可运行和数学声明是三件不同的事。

## 决策

采用以下五层：

1. `domain`：系统、状态、动作、控制器、证书、modifier 和 claim 的纯领域契约；
2. `methods`：控制器、证书方法和可选 baseline 实现；
3. `application`：episode、evaluation、guard、指标和 telemetry 编排；
4. `adapters`：数据、存储、exact、ROS 和 Gazebo 外部适配；
5. `entrypoints`：CLI 与未来 ROS composition root。

依赖只向内：`methods` 与 `application` 指向 `domain`；`adapters` 可以依赖
application/domain/method 端口；`entrypoints` 负责最外层组装。`domain` 不认识
Torch、ROS、Gazebo 或磁盘路径。learned 训练因为直接涉及数据集、文件、绘图和
框架运行，位于 `adapters/training/`，不让 `methods` 反向依赖 adapter。

同时决定：

- learned RGB-D 移到 `methods/baselines/` 与 `configs/baselines/`；
- certificate evaluator 与 action modifier 分开；
- 证书离线求解/验证，在线只加载、兼容校验和评估；
- `CertificateArtifact` 与 learned `ModelManifest` 分开治理；
- 一个 `ScenarioSpec` 驱动 exact、Gazebo、RViz、metrics 和快照；
- ROS2 只保留 `cbf_rover_description`、`cbf_rover_sim`、
  `cbf_rover_runtime` 三个拥有实际职责的包；
- 第一条数学链从研究契约和 exact backend 开始，不从 ROS 节点开始。

## 为什么 learned 降为 baseline

learned RGB-D 的 CBF-like/CLF-like 输出由有限数据和网络预测产生。当前没有全局
模型误差界、独立 safe-set 验证或形式证明。它对经验比较仍有价值，但不能决定
领域接口，也不能成为新证书方法必须适配的默认架构。

将其降为 baseline 保留了可比较性，同时让新的 polynomial/q-SBC 方法不用导入
图像、Torch、checkpoint 或 recurrent state。

## ROS2 决策

机器人几何、Gazebo 资源和运行节点分成三个包。不会为未来 interface、
visualization 或 bringup 预建空包。`cbf_rover_runtime` 中保留的 command relay
是默认禁用的 external command message adapter，不代表证书 runtime 已完成。

`cbf_rover_sim/worlds/base.world` 不含研究障碍；应用层把根目录 ScenarioSpec
投影为 obstacle spawner 输入。

## 结果

收益：

- 证书公式和运行环境解耦；
- learned 与 formal/analytic 方法可以用统一应用协议比较；
- exact 和 Gazebo 共享场景但保留不同证据边界；
- 每次 certificate 更新都能通过 artifact hash 和独立验证追溯；
- ROS 包数量与实际职责一致。

代价：

- 旧导入路径和旧配置路径不再兼容；
- 首个端到端闭环需要先完成研究契约、专用 evaluator 和 exact adapter；
- ScenarioSpec 投影器完成前，Gazebo 不再内置示例障碍；
- 现有 learned CLI 仍然是 baseline 工具，不能代表项目主入口已经 math-first。

## 未选择的方案

### 保留旧目录并继续补节点

短期移动较少，但会继续让 learned 数据流塑造整个系统，拒绝。

### 立即拆成大量 ROS 包

interface、visualization、bringup 尚无稳定职责。空包会增加维护面并伪装进度，
拒绝。

### 把所有证书字段放进一个万能 evaluation

q-SBC、确定性 CBF 和 learned diagnostic 的定理语义不同。公共 contract 保持小，
专用字段由具体方法扩展，拒绝万能类型。

### 同时迁移 Gazebo 代际和数学核心

两条高风险变化会混淆故障来源。本轮只清理包职责，仿真平台升级另立决策，暂缓。

## 验证边界

本 ADR 记录架构决策，不是运行证据。本轮没有在本机运行测试、ROS2 构建或
Gazebo。
