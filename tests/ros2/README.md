# Server-only ROS2 and Gazebo tests

本目录预留给 ROS2、Gazebo 与仿真桥接回归测试。所有测试只在安装项目规定 ROS2/Gazebo 版本的服务器环境运行，本机不启动 `colcon`、ROS2 节点或 Gazebo。

计划覆盖：

- 三个仿真包的构建、安装与 launch 文件解析；
- 固定障碍物 world、机器人描述、传感器话题和差速驱动接口；
- bridge 生命周期、参考动作输入、安全过滤输出与已执行动作回传；
- 缺少外部模型工件时 learned controller 保持禁用并 fail closed；
- 固定场景与随机种子的碰撞、干预和证书违反指标回归。

项目不包含实车包，也不安排实车测试。ROS2/Gazebo 结果仅支持限定场景下的经验性结论，不得表述为形式化安全证书。
