# Server-only ROS2 and Gazebo tests

本目录预留给 ROS2、Gazebo 与运行时 adapter 的服务器回归测试。本机不运行
`colcon`、ROS2 节点或 Gazebo。

计划覆盖：

- `cbf_rover_description`、`cbf_rover_sim`、`cbf_rover_runtime` 的构建、安装和 launch 解析；
- 固定 world、机器人描述、传感器话题与差速驱动接口；
- external command relay 默认不启动、参数与 topic 对齐、消息转换和正常关闭零
  命令；该 adapter 没有输入安全验证、watchdog 或 fail-closed 能力；
- 固定场景和随机种子的碰撞、动作修改与证书违反指标回归。

项目不包含实车运行包，也不安排实车测试。当前没有已运行或已通过的 ROS2/Gazebo
结果；服务器仿真结果仅支持限定环境下的经验性结论。
