# 服务器验证门禁

本文是后续执行清单，不是已通过记录。本轮没有在本机运行测试、包导入、模型、
训练、ROS2、Colcon、Gazebo 或实验。

## 1. 固定运行身份

- 记录 Git revision、branch/tag 和工作区状态；
- 记录 OS、Python、ROS2、Gazebo、solver、CUDA、GPU 和框架版本；
- 确认 `CBF_ROVER_ARTIFACT_ROOT` 与 `CBF_ROVER_RUN_ROOT` 位于仓库外；
- 独立复核 `接手的原始文件/` 仍只有最初三份压缩包及预期 SHA-256；
- 保存所有解析配置的内容哈希。

工作区脏、配置快照缺失或外部根目录未设置时，不运行正式实验。

## 2. 数学契约门禁

在实现或运行首个 q-SBC 前：

- 用户确认 `docs/mathematics/RESEARCH_CONTRACT.md`；
- system、controller、scenario 和 noise 配置冻结并计算哈希；
- theorem、证明义务、概率语义和允许声明措辞完整；
- solver、容差、缩放和 independent validator 已指定。

未完成时只能运行架构/诊断代码，不能生成带 formal 标签的 artifact。

## 3. Python 静态与测试

在仓库根：

```bash
python -m pip install -e ".[server-dev]"
python -m pytest
python -m pytest tests/provenance
```

默认 unit/math 路径不应因 Torch、HDF5 或 ROS 未安装而无法收集。随后只有在执行
learned baseline/integration 阶段时再安装：

```bash
python -m pip install -e ".[learned-rgbd,server-dev]"
```

`tests/integration/` 当前只有验收边界说明；加入实际测试模块后再运行该目录，不能
把“无测试可收集”记为通过。

需要覆盖：

- domain 不导入 Torch/ROS/Gazebo；
- state/action/schema 的单位、顺序与非有限值门禁；
- evaluator 不修改 action；
- modifier 不可行时 environment step 未调用；
- artifact/config/hash 不匹配 fail closed；
- ScenarioSpec 被 exact/Gazebo/RViz/metrics 同源解析；
- learned baseline 缺模型时 fail closed；
- 旧导入路径没有残留。

命令是否通过必须在服务器保存实际输出；本文件不预先声称结果。

## 4. CertificateArtifact

对每个候选证书：

1. 记录 system/controller/scenario/noise ID 与 SHA-256；
2. 记录 state order、units、frame、domain 和 `dt`；
3. 记录表示、变量顺序、系数或外部系数文件哈希；
4. 保存 solver 版本、终止状态、容差和原始证据；
5. 使用独立 validator 复核；
6. 重新计算 artifact 自身 SHA-256；
7. 由验证结果推导 claim eligibility。

solver success 但 independent validation 未通过时，不得激活。

## 5. Exact backend

- 对 3–5 个手算状态核对一步动力学；
- 核对 initial/goal/unsafe/domain 边界；
- 核对 `W`、一步期望、残差和容差；
- 固定 seed 重复运行并比较逐步 trace；
- 注入不兼容 hash、错误 state order、NaN 和域外状态；
- 验证 monitor-only 不改变 nominal action；
- 验证 replay 与在线 evaluator 结果一致。

exact 通过表示实现与契约一致，不表示定理自动成立。

## 6. ROS2 与 Gazebo

在服务器构建：

```bash
cd simulation/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

启动基础仿真：

```bash
ros2 launch cbf_rover_sim simulation.launch.py
```

验证：

- `cbf_rover_description` 只有几何和 frame，无仿真插件；
- `cbf_rover_sim/worlds/base.world` 不含研究障碍；
- application runner 从根 ScenarioSpec 生成三障碍 projection 并注入 spawner；
- Gazebo/RViz/metrics 解析后的几何与 ScenarioSpec 快照一致；
- odometry、RGB、depth、collision、clock 和 command topics 与配置一致；
- reset 同时重置世界、controller、evaluator、modifier 和随机状态；
- requested/modified/commanded/executed action 可区分；
- watchdog、时间同步、NaN、stale state 和 shutdown 均 fail closed。

Gazebo Classic 插件库、bumper 消息/topic 和 RGB-D 行为必须在目标安装上确认。

## 7. ROS runtime 边界

`cbf_rover_runtime` 当前 external command relay 默认禁用。验证它时必须明确：

- relay 不验证命令、模型或证书，也不修改动作；
- `start_external_command_relay` 未显式启用时不创建节点；
- relay YAML 的 input/output topic 与节点实际参数一致；
- 收到 `TwistStamped` 后只做消息类型转换；
- 正常关闭时发送一次零 `Twist`，但这不等于 watchdog 或 fail-safe。

真正的 certificate runtime 接入后，需要单独验证 state encoder、evaluator、
modifier、guard、telemetry 和 actuator chain。

## 8. Learned model 门禁

- 在服务器重新训练或从可信存储显式注入五个模型；
- 填写 `artifacts/models/manifest.yaml` 的 URI、SHA-256、字节数、来源和兼容字段；
- 独立计算哈希；
- 实际加载每个模型并执行最小输入输出检查；
- 人工审查后才把槽位改为 `verified`。

模型 verified 只覆盖身份与加载兼容性。learned 结果仍为 empirical。

## 9. 结果验收

- 所有 run 都有不可覆盖 ID、完整配置/环境/制品快照；
- trace、episode summary、聚合表和图可以双向追溯；
- exact 与 Gazebo 结果分开报告；
- success、collision、unsafe entry、task performance 和 intervention 一起报告；
- formal/bounded/empirical/diagnostic 措辞与实际证据一致。

测试通过、模型加载、ROS2 构建和 Gazebo 冒烟各自只证明对应链路。任何一步都
不能单独替代数学证明。
