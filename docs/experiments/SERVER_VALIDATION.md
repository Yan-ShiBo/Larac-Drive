# 服务器验证门禁

本文是后续执行清单，不是已通过记录。本次重构没有在本机运行测试、模型、ROS2、
Colcon、Gazebo、训练或实验。

## 1. 静态环境记录

- 记录 Git revision 和工作区状态；
- 记录 OS、ROS2、Gazebo、Python、CUDA、GPU 和框架版本；
- 确认 `CBF_ROVER_ARTIFACT_ROOT` 与 `CBF_ROVER_RUN_ROOT` 指向仓库外目录；
- 确认原始归档目录仍只包含最初压缩包。

## 2. 制品准备

- 在服务器重新训练，或从可信外部存储注入；
- 为五个槽位填写 URI、SHA-256、字节数、训练运行和兼容信息；
- 独立计算哈希并与清单比对；
- 实际加载每个模型并执行最小输入输出检查；
- 审查通过后才把槽位改为 `verified`。

任何槽位缺失或验证失败时，learned RGB-D 必须 fail closed，不能自动使用随机
权重、旧路径或同名文件。

## 3. 核心与 ROS2

- 在仓库根安装并测试：

  ```bash
  python -m pip install -e ".[learning,server-dev]"
  python -m pytest
  ```

- 构建新的 ROS2 工作区：

  ```bash
  cd simulation/ros2_ws
  source /opt/ros/humble/setup.bash
  rosdep install --from-paths src --ignore-src -r -y
  colcon build --symlink-install
  source install/setup.bash
  ```

- 启动仿真结构检查：

  ```bash
  ros2 launch cbf_rover_gazebo simulation.launch.py
  ```

- 验证图像、深度、里程计、碰撞和命令接口；
- 验证 episode reset 同时重置世界、方法状态和 MDRNN 隐状态；
- 在 `publish_command: false` 下完成无驱动链路检查。

上述命令目前只是后续门禁，不是通过记录。尤其需要在目标 ROS2 Humble/Gazebo
Classic 环境确认 depth camera 与 bumper 插件的库名、消息类型和 topic。

## 4. 对照实验

- 先运行 reference policy；
- 再用完全相同的场景、初始状态和 seed 运行 learned 过滤；
- 检查所有逐步字段与 episode 汇总均完整；
- 对 NaN、时间不同步、模型维度错误、不可行过滤和异常退出单独计数；
- 保存配置、环境、制品和结果快照。

## 5. 验收措辞

服务器通过加载、构建或仿真冒烟，只证明相应链路在记录环境中可运行。成对实验
结果只能作为经验证据；没有额外数学证明时，不得声称形式化安全。
