# 2026-07-28 清洁重构记录

> 历史快照：本文记录 math-first 整体重构之前的清洁架构，其中旧目录名只用于
> 说明当时状态。当前架构与路径见
> [`MATH_FIRST_REBUILD_2026-07-28.md`](MATH_FIRST_REBUILD_2026-07-28.md)。

## 决策与边界

本轮按用户明确确认执行“重新设计全部架构并删除旧代码、旧结果、继承权重和
旧 Git/LFS”。唯一不可变边界是 `接手的原始文件/`：该目录不新增、不删除、
不改名、不改内容，也不作为运行时路径。

项目主线固定为：

- Gazebo/ROS2 仿真，不包含实车驱动；
- 障碍证书与安全过滤研究；
- 方法按纵向包组织，便于持续新增和替换；
- learned CBF/CLF-like 只声明为经验型证据；
- 真实测试、模型、训练、ROS2 构建和 Gazebo 运行留到服务器。

## 原始归档基线

清理前后均确认目录只有三个文件、没有子目录，最终 SHA-256 为：

| 文件 | 字节数 | SHA-256 |
|---|---:|---|
| `rgbd_safe_method.7z` | 42,052 | `AB2F0A0ECEB97D78CB825E04ADEAC2BEB64D27CD3F53A937FF4475708E3893B0` |
| `yzh-skills.7z` | 44,717,933 | `72DE126DE7A22EF0E6E6B82838710D2DD169822620E1B27FB32D0F886E2ECA21` |
| `yzh1-杨子恒的所有原始代码.7z` | 860,136,841 | `405C1A810D72915F14BA759D282E79CC1D8C2D585EE787A591BE029E0FE66A07` |

归档由 `.gitignore` 排除，不把接手现场证据混入活动源码或 Git 历史。

## 删除内容

从活动工作区移除了以下类别：

- `legacy/` 的旧 ROS1、RGB-D 副本、历史实验和训练结果；
- 旧 `ros2_ws/`，包括重复仿真实现、实车/串口兼容层和五个活动继承权重；
- 旧顶层 `scripts/`；
- `src/cbf_rover` 下八个旧横向包；
- 四组旧配置、旧 handover/接口/服务器文档和过渡模型注册表；
- Ruff 缓存；
- 无 commit、无 refs、无 remote，但包含 Git LFS 对象与 filter 的旧 `.git/`。

前 24 个精确目标共 502 个文件、1,189,747,473 B；旧 `.git/` 另有
172 个文件、474,085,002 B。总计从工作区移出 1,663,832,475 B，约
1.55 GiB。

这些内容通过 Windows 回收站做可恢复删除；回收站清空前仍可恢复。清空后，
历史代码和权重只可从永久只读原始归档重新提取，且必须解压到归档目录之外。

## 新活动架构

- `src/cbf_rover/core`：不依赖 Torch、ROS 或文件路径的公共契约；
- `src/cbf_rover/methods/learned_rgbd`：VAE、MDRNN、learned CBF/CLF、
  安全过滤和训练；
- `src/cbf_rover/experiments`：episode 生命周期、runner、记录与指标；
- `src/cbf_rover/datasets`、`policies`、`cli`：数据、参考策略接口和统一命令；
- `simulation/ros2_ws/src`：description、Gazebo、ROS lifecycle bridge 三包；
- `configs`：method、policy、scenario、experiment、runtime 五个配置域；
- `artifacts/manifests`：仓库外制品身份、来源和激活门禁；
- `tests`：unit、provenance、integration、ros2 四层服务器测试。

learned 推理只能从 `artifact_set.status: ready` 且相关 slot 为 `verified` 的
manifest 构造 checkpoint set。文件必须位于 `CBF_ROVER_ARTIFACT_ROOT` 内，
字节数与 SHA-256 匹配，并使用 Torch `weights_only=True` 加载。当前五个槽位
全部为 `missing`，方法与实验保持禁用。

## 保留的仿真资产

只从旧 ROS2 工作副本迁移了无法由文本重建的 7 个仿真资产，并逐文件验证
源—目标大小和 SHA-256：

| 资产 | SHA-256 |
|---|---|
| `ground.dae` | `463BF5A12D4E98B0B8ED20A3FF08FA27353A5606F485C379B3CB8D9EE92B5919` |
| `tile.jpg` | `AFE399FC3CADBB2C61E7506C98465ACD6C4497A8218DE2314DD6BA4EF45CE282` |
| `road_line.png` | `2B9392FC80CB535E2FEB37466CFD3C3BC59F8ED955D499BE45DD1213BB1504F6` |
| `road_line1.png` | `9E3975CA22932188E91A40A6AF987CD5E0FF2B4A331215745D692F10E67D48B4` |
| `road_mask_ring.png` | `C17D4B17D2FE0BEFA817B4A187AF9E056E96745E5458AD907E5A92D280510CBE` |
| `road_midline.png` | `8ED4FA5046F5533E1A3833A89989769036471EE8FA6D6B2D37A0797DCFDE62FD` |
| `tile_mask_line.png` | `2843B611A177B02D0293C3BAD563D00D902BD66A5599FE57A66F58E0C53C8EE2` |

## 验证证据与限制

删除后的静态检查包括：

- Python AST、TOML、YAML、XML/Xacro/SDF/World/DAE 解析；
- 活动 Markdown 本地链接；
- 旧包名、个人绝对路径、动态修改 Python 搜索路径和不安全 Torch fallback；
- 仓库内模型权重、删除目标残留和原始归档哈希；
- 仿真 world、RGB-D、碰撞 topic 和复制资产边界。

静态检查通过不等于测试通过。本轮没有运行 pytest、项目 import、模型加载、
训练、ROS2、Colcon 或 Gazebo。实验 YAML 与 recording 仍是服务器待接入的
执行骨架；depth camera、bumper 插件、topic/message 和完整实验流水线必须按
[服务器验证门禁](../experiments/SERVER_VALIDATION.md) 在目标服务器验证。

## 该阶段当时的 Git 状态

旧 Git/LFS 已移除，随后初始化空的 `main` 仓库。新仓库没有 commit、暂存、
remote 或 local LFS filter；模型二进制由 `.gitignore` 和外部 artifact 规则
管理。
