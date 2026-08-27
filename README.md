# humanoid_driver_runtime

这是平台统一驱动节点所在的包，也是 `RobotDriverPlugin` 的唯一加载者。它负责：

- 订阅统一命令 `/hc_teleop/joint_cmd`；
- 调用已加载驱动的 `writeJointCommand()`；
- 调用驱动读取真实状态并发布 `/hc_teleop/joint_states`；
- 管理驱动的配置、连接、激活、停止生命周期；
- 执行命令看门狗并发布 `/diagnostics`。

`humanoid_driver_interface` 只定义驱动必须实现的 C++ 规范，本包才是运行这个规范的节点。
`humanoid_motion_server` 不知道具体机器人，也不会直接加载驱动。

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

## 启动

单独启动模拟驱动节点：

```bash
ros2 launch humanoid_driver_runtime bringup.launch.py
```

接入机器人已有 Topic 的完整说明见 `docs/adding_ros_topic_robot.md`。
串口、CAN、EtherCAT 或厂商 SDK 驱动的独立 pluginlib 包导出方式见
`docs/adding_driver_plugin.md`。
