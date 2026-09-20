# humanoid_driver_runtime

这是平台设备运行层，同时提供彼此独立的机械臂和夹爪运行节点：

- 订阅统一命令 `/hc_teleop/joint_cmd`；
- 调用已加载驱动的 `writeJointCommand()`；
- 调用驱动读取真实状态并发布 `/hc_teleop/joint_states`；
- 管理驱动的配置、连接、激活、停止生命周期；
- 执行命令看门狗并发布 `/diagnostics`。

`humanoid_gripper_runtime_node` 以相同的生命周期加载 `GripperDriverPlugin`，平台侧使用
`/hc_teleop/gripper_commands` 和 `/hc_teleop/gripper_states` 两个具名 `JointState` 话题。夹爪
插件可以控制一个或多个夹爪，但不会进入机械臂的 14 轴运动关节集合。

`humanoid_driver_interface` 只定义驱动必须实现的 C++ 规范，本包才是运行这个规范的节点。
`humanoid_motion_server` 不知道具体机器人，也不会直接加载驱动。

## 机械臂反馈超时与恢复

机械臂命令必须携带非零、递增且与节点 ROS 时钟一致的 `header.stamp`，
`command_max_age_ms` 默认 100 ms。运行层按关节保留最新待执行目标，再按配置的组下发，
左右臂互不覆盖；重复、乱序、过期命令不下发，也不刷新看门狗。有效期会在调用插件前再次检查。

`feedback_max_age_ms` 默认 100 ms，按插件的 `JointState.sample_time` 检查实际采样年龄。
同一采样只发布一次，平台时间戳保留其年龄。ROS 话题驱动从厂商 `JointState.header.stamp`
计算采样时刻；零、冻结、乱序、过期时间戳不能刷新反馈有效期。自定义硬件插件同样必须
提供实际采样时间，不能在重复读取缓存时改成当前时间。此检查无法代替底层 CAN 回包有效性检查。

已连接、已激活的驱动报告 `kStale` 时，运行层进入 `feedback_waiting`：保持位置、暂停运动
命令，并继续读取反馈。完整且有效的反馈恢复、驱动健康检查通过后，运行层用当前实测位置
重新建立保持目标，恢复平台关节反馈和末端 FK；后续运动使用新收到的命令，不重放旧目标。
此逻辑也适用于启动后反馈暂时中断、或首帧晚于启动宽限期到达的情况。

驱动报告 `kError`、断开连接、数据校验失败、异常或实际命令发送失败，仍进入
`driver_fault_latched`。`kStale` 只用于反馈缺失/过期；插件应把硬件故障报告为 `kError`。

`/diagnostics` 增加 `feedback_waiting`、`feedback_interruptions` 和 `feedback_recoveries`。
故障锁定会明确显示为 ERROR；反馈等待显示为 STALE。等待、恢复和锁定日志共用 30 秒间隔，
以累计计数汇总，恢复事件即使发生在间隔内也会在之后补报，不逐帧写日志。

## 关节命令与实测运动日志

默认 INFO 日志按配置的 `vendor_joint_groups` 分组记录状态变化：

- `joint_command.<组>`：驱动成功接受该组关节命令时进入 `receiving`，超过命令看门狗间隔
  未再接受新命令时进入 `stopped`。接收命令不代表硬件已运动。
- `joint_feedback.<组>`：实测关节为 `moving`、`stationary` 或 `unavailable`。对关节名称
  做匹配，不依赖反馈数组顺序；任一关节累计变化达到 0.005 rad 算运动，连续 0.5 秒未出现
  此变化算静止。反馈丢失记录 `unavailable`，恢复后用当前实测位置重新建立比较基线。

首次记录基线，稳定状态不重复输出。每组每项最多每秒一条，快速切换合并为变化路径和计数，
最多保留 8 次状态；超过部分给出省略次数。只保存固定数量状态，不保存每帧关节轨迹。
运动阈值仅用于诊断日志，不参与命令、安全和看门狗判断。

配合接收端的 `input.transport`、`input.accepted`、`arm.<通道>.output`，可回看一次操作
是否收到手柄数据、是否发出末端目标、是否形成驱动命令，以及关节反馈是否实际变化。

## 两种机器人都如何接入

1. 机器人已经提供 ROS 2 关节状态和命令 Topic：使用内置
   `humanoid_driver_runtime/RosTopicRobotDriver`，在 YAML 中填写原生 Topic 和关节映射。
2. 机器人只提供 C/C++ SDK 或专用驱动：实现一个 `RobotDriverPlugin` 插件，由本节点加载。

两种方式对平台都只暴露同一对 Topic。上层程序不应访问机器人原生 Topic 或 SDK，也不应
直接发布 `/hc_teleop/joint_cmd`；关节命令的权威发布者是 `humanoid_motion_server`。

## 关节映射放在哪里

映射属于机器人驱动配置，放在本包或机器人专属驱动包的 YAML 中：

```yaml
joint_names: [right_arm_joint_1, right_arm_joint_2]
vendor_joint_names: [Joint1_R, Joint2_R]
vendor_joint_groups: [right_arm, right_arm]
vendor_to_logical_scales: [1.0, 1.0]
vendor_to_logical_offsets_rad: [0.0, 0.0]
```

平台名保持不变，换机器人时只换 `driver_params_file` 和必要的插件，不改运动服务接口。

`plugin_class` 是运行时插件选择点。新驱动放在独立 ROS 包中，通过 pluginlib 导出
`humanoid_driver_interface::RobotDriverPlugin`；本节点会按 YAML 中的类名加载，不需要把
机器人类型写进本节点源码。若厂商已经提供 `JointState` 状态/位置命令 Topic，直接复用
`humanoid_driver_runtime/RosTopicRobotDriver`，只增加映射 YAML。

对于按组接收 `Float64MultiArray` 的位置控制器，可以使用
`command_topic.<vendor_group>=/controller/commands`，每个组配置一个话题，
与单个 `command_topic`（`JointState` 模式）互斥。数组顺序就是映射配置中该组的关节顺序；
只发布被更新的组，局部更新保留同组其他关节的目标，首次目标和停止保持值来自实测反馈。
示例见 `config/group_position_driver.yaml`。这种模式只输出位置，适用于接口符合此约定的机器人。

## 启动

单独启动模拟驱动节点：

```bash
ros2 launch humanoid_driver_runtime bringup.launch.py
```

接入机器人已有 Topic 的完整说明见 `docs/adding_ros_topic_robot.md`。
串口、CAN、EtherCAT 或厂商 SDK 驱动的独立 pluginlib 包导出方式见
`docs/adding_driver_plugin.md`。
