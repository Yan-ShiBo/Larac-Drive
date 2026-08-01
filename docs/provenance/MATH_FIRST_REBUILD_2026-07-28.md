# 2026-07-28 math-first 整体重构记录

## 基线与分支

重构开始前先固定了可恢复基线：

| 项目 | 值 |
|---|---|
| baseline commit | `1e5a8d8` (`chore: preserve clean simulation-first baseline`) |
| baseline branch | `main` |
| baseline tag | `pre-math-first-refactor` |
| rebuild branch | `codex/refactor-math-first-architecture` |

该基线保留前一阶段清洁架构，允许当前整体目录重构通过 Git 比较和恢复。本记录不
代表当前工作区已经提交。

## 迁移目标

目录从 learned RGB-D 驱动的横向组织，改为研究目标驱动的五层：

```text
domain -> methods -> application/adapters -> entrypoints
```

依赖方向实际为 entrypoints/application/adapters/methods 向 domain 内聚。
learned RGB-D 降为可选 baseline；首个证书主线从研究契约、数学 evaluator 和
exact backend 开始。

## 迁移边界

本轮授权整体替换活动目录，但保持以下边界：

- `接手的原始文件/` 永久只读，不解压、不修改、不写入；
- 不恢复已经删除的旧结果、权重、ROS1 或实车驱动；
- 不运行本地测试、模型、训练、ROS2、Colcon 或 Gazebo；
- 不伪造 q-SBC 公式、证书系数、solver success 或 independent validation；
- 大模型、数据和运行结果位于项目内的 Git 忽略目录（`artifacts/payloads/`、
  `data/`、`runs/`）；
- 只保留拥有当前具体职责的三个 ROS2 包。

## 旧目录退出与新职责

| 退出路径 | 新路径/处理 | 新职责 |
|---|---|---|
| `src/cbf_rover/core/` | `src/cbf_rover/domain/` | 传输无关的 state/action/system/certificate/claim contract |
| `src/cbf_rover/policies/` | `src/cbf_rover/methods/controllers/` | nominal controller 实现 |
| `src/cbf_rover/experiments/` | `src/cbf_rover/application/` | episode、runner、metrics、telemetry |
| `src/cbf_rover/datasets/` | `src/cbf_rover/adapters/datasets/` | 数据格式和 I/O 适配 |
| `src/cbf_rover/cli/` | `src/cbf_rover/entrypoints/cli/` | 薄命令入口 |
| `src/cbf_rover/methods/learned_rgbd/` | `src/cbf_rover/methods/baselines/learned_rgbd/` | 默认禁用的经验 baseline |
| learned `training/` | `src/cbf_rover/adapters/training/learned_rgbd/` | 数据、文件、绘图和训练运行适配 |
| `cbf_rover_gazebo` | `cbf_rover_sim` | base world、仿真插件和场景投影入口 |
| `cbf_rover_ros` | `cbf_rover_runtime` | ROS2 运行适配边界 |
| `configs/methods/`、`configs/policies/` | `configs/baselines/learned_rgbd/` | learned 方法与 depth PPO 配置 |
| `configs/runtime/` | `configs/runtimes/` | 服务器、ROS2、设备和外部存储 |
| `artifacts/manifests/` | `artifacts/models/` 与 `artifacts/certificates/` | 模型身份与数学证书分开治理 |
| `docs/methods/` | `docs/mathematics/` 与 `docs/baselines/` | 证明语义与经验 baseline 分离 |

新增 `configs/systems/`、`controllers/` 和 `certificates/`，但在首个研究契约确认
前不放置活动数学配置。

## 场景权威化

`configs/scenarios/static_obstacles.yaml` 升级为逻辑 ScenarioSpec，集中记录：

- `world` frame 和 SI units；
- domain、initial/goal/unsafe sets；
- conservative rover footprint 与 inflation；
- 三个障碍的 ID、中心和尺寸；
- exact/Gazebo backend 绑定；
- episode、seed、终止条件和 topics。

`cbf_rover_sim/worlds/base.world` 只保存基础环境，不再定义研究障碍。ScenarioSpec
将作为 exact、Gazebo obstacle projection、RViz markers、metrics 和 run snapshot
的唯一真源。

## 制品分离

`artifacts/models/` 保留 learned 模型 manifest/schema。身份、完整性和加载检查
不等于安全证明。

`artifacts/certificates/` 新增 certificate artifact schema，要求记录：

- system/controller/scenario/noise 哈希；
- state schema、顺序、单位、frame 和 `dt`；
- 表示、变量顺序和系数；
- solver 状态、版本、容差和证据；
- independent validation；
- assumptions、failure conditions 和 derived claim eligibility。

当前没有 certificate artifact。

## 研究契约

`docs/mathematics/RESEARCH_CONTRACT.md` 明确把首个 q-SBC 的系统、controller、
集合、噪声、完整公式、概率语义和求解/验证方法留给用户确认。占位未确认前，不
生成证书或概率声明。

## 原始归档基线

本次记录沿用前一阶段的三份归档基线：

| 文件 | 字节数 | 记录的 SHA-256 |
|---|---:|---|
| `rgbd_safe_method.7z` | 42,052 | `AB2F0A0ECEB97D78CB825E04ADEAC2BEB64D27CD3F53A937FF4475708E3893B0` |
| `yzh-skills.7z` | 44,717,933 | `72DE126DE7A22EF0E6E6B82838710D2DD169822620E1B27FB32D0F886E2ECA21` |
| `yzh1-杨子恒的所有原始代码.7z` | 860,136,841 | `405C1A810D72915F14BA759D282E79CC1D8C2D585EE787A591BE029E0FE66A07` |

本子任务没有重新计算这些哈希，也没有修改或写入归档目录。重构完成后由主代理
再次复核文件数、字节数和 SHA-256，并把实际结果作为最终证据。

## 未执行内容

本轮没有执行：

- Python import 或 CLI；
- pytest、ruff 或其他测试；
- model loading/inference/training；
- solver 或 certificate synthesis；
- ROS2/Colcon build；
- Gazebo launch、RViz 或 topic 验证；
- exact 或 Gazebo episode。

文档、YAML、目录和代码存在只表示静态结构，不是运行通过记录。

## 完成复核

主代理在合并并行改动后完成了以下只读或文本级复核：

- [x] 13 个退出目录均不存在；活动 ROS2 目录恰为三个包；
- [x] 12 个 XML/Xacro/SDF/world/model.config/DAE 文件通过 XML 读取；
- [x] 9 个 YAML 无 tab、奇数缩进或尾随空格，5 个仓库内 YAML 引用均存在；
- [x] 33 个 Markdown 文件中的 56 个本地链接均存在；
- [x] 70 个 Python 文件无超过 120 字符的行或 tab；
- [x] methods/application 无对外层 package 的反向导入，活动源码无旧导入；
- [x] base world 无研究障碍，launch/spawner 无 `box_1/2/3` 默认几何；
- [x] ScenarioSpec 的 `world_name: cbf_rover_world` 与 SDF world 匹配；
- [x] 永久归档目录之外的仓库内无模型权重文件或 reparse point；
- [x] 永久归档仍为 3 个文件、0 个目录，字节数与三个 SHA-256 全部匹配基线；
- [x] `git diff --check` 无错误，改动范围是本次授权的整体架构替换；
- [x] 明确未运行 Python、pytest、ruff、模型、训练、ROS2、Colcon 或 Gazebo。

这里的 YAML/Python 检查是文本和依赖边界检查，不是 parser、import 或运行测试。
所有可执行验收仍保留到服务器。
