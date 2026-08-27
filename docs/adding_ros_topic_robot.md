# 接入一个 ROS 2 Topic 机器人

平台固定使用下面两个接口，关节数量由消息中的 `name` 和数组长度决定：

| 方向 | 固定平台 Topic | 类型 |
| --- | --- | --- |
| 机器人 -> 平台 | `/hc_teleop/joint_states` | `sensor_msgs/msg/JointState` |
| 平台 -> 机器人 | `/hc_teleop/joint_cmd` | `sensor_msgs/msg/JointState` |

`humanoid_driver_runtime_node` 和 `humanoid_driver_runtime/RosTopicRobotDriver` 共同把厂商自己的 ROS 2 关节 topic 接到这两个固定接口。上层只看逻辑关节名和 SI 单位；厂商 topic、关节名、数组顺序、正反方向和零位偏置都留在配置中。

```
/hc_teleop/joint_cmd [逻辑名, rad]
  -> humanoid_driver_runtime_node -> RosTopicRobotDriver
  -> 厂商命令 topic [厂商名, 厂商坐标]

厂商状态 topic [厂商名, 厂商坐标]
  -> RosTopicRobotDriver -> humanoid_driver_runtime_node
  -> /hc_teleop/joint_states [逻辑名, rad]
```

它不要求固定关节数量：单臂 6 轴配置 6 条 mapping，双臂 14 轴配置 14 条 mapping。命令由 `joint_names` 关联，不靠数组的固定下标。一个命令还可以只包含已配置关节的子集。

## 适用的厂商接口

该现成插件要求厂商两端都使用 `sensor_msgs/msg/JointState`：

| 方向 | 数据要求 |
| --- | --- |
| 厂商 -> 平台状态 topic | `name`、`position` 必填；`velocity`、`effort` 可同时省略。每条有效状态必须包含全部已配置的厂商关节。 |
| 平台 -> 厂商命令 topic | `name`、`position` 必填。`velocity`、`effort` 按上游命令可选；此平台将此消息定义为位置设定值，厂商端应按名称执行或保持对应位置。 |

所有平台逻辑量均使用 rad、rad/s、rad/s² 和 N·m。供应商若使用度、轴向相反或不同的零位，使用每个关节的 `vendor_to_logical_scales` 和 `vendor_to_logical_offsets_rad` 转换：

```text
logical_position_rad = scale * vendor_position + offset_rad
vendor_position = (logical_position_rad - offset_rad) / scale
```

例如，厂商的角度值为度时 `scale = 0.017453292519943295`；反向弧度轴用 `scale = -1.0`。`scale` 不可为零。

厂商状态 subscription 使用 `SensorDataQoS`，可连接 best-effort 或 reliable 状态 publisher；厂商命令 publisher 使用 depth 10 的 reliable/volatile QoS。平台反馈 publisher 和平台命令 subscription 都使用 depth 10 的 reliable/volatile QoS。

如果厂商的原生消息不是 `sensor_msgs/msg/JointState`（如专有 `.msg`、CAN 帧或 `Float64MultiArray`），**仅靠 YAML 无法可靠解析任意数据模式**。这种情况下保留相同的 `RobotDriverPlugin` 合约，新增一个很薄的厂商 codec 插件，将原生消息转换为同样的逻辑关节集合即可；控制、仲裁与安全层不需要改动。

## 通用桥配置

下面是两关节示例。换成单臂 6 轴或双臂 14 轴时只需增减四组 mapping 数组，不改变平台 topic 或代码。

```yaml
humanoid_driver_runtime:
  ros__parameters:
    plugin_class: humanoid_driver_runtime/RosTopicRobotDriver
    command_watchdog_ms: 100.0
    control_frequency_hz: 100.0

    platform_joint_state_topic: /hc_teleop/joint_states
    platform_joint_command_topic: /hc_teleop/joint_cmd

    joint_names: [left_joint_1, left_joint_2]
    vendor_joint_names: [joint_01, joint_02]
    vendor_joint_groups: [left_arm, left_arm]
    vendor_to_logical_scales: [1.0, -1.0]
    vendor_to_logical_offsets_rad: [0.0, 0.0]

    plugin_parameters:
      - state_topic=/vendor_robot/feedback/joints
      - command_topic=/vendor_robot/control/joint_positions
      - state_timeout_s=0.25
      - startup_grace_s=3.0

```

`state_timeout_s` 是最近一条有效状态回调的最大允许间隔。超过后读/写会报告通信故障并触发平台安全停止。`startup_grace_s` 只用于节点刚进入 executor 前等待第一条状态，避免订阅还未得到调度就被错误锁死；超过该时间仍无状态同样是故障。

与驱动 mapping 一起，还要在生产运动配置中为这台机器人提供其实际存在的关节组、限位、URDF 和运动学配置。单臂机器人不应伪造 `right_arm`；请求不存在的组应由上层正常拒绝。

## 添加新机器人的步骤

1. 记录厂商状态/命令 topic、消息类型、每个关节名称、单位、正方向、零位以及安全停机语义。
2. 若两端都是 `JointState`，复制上述 YAML，填入 mapping 和四个插件参数；不改 C++。
3. 启动 `humanoid_driver_runtime_node`，确认厂商状态已经转换到 `/hc_teleop/joint_states`。
4. 以低速安全目标向 `/hc_teleop/joint_cmd` 发布命令，确认桥接后的厂商命令名称、单位和方向正确。
5. 核实反馈是测量值而不是命令回显；拔掉/暂停厂商状态后确认 `state_timeout_s` 触发停止。
6. 再补齐该机器人的 URDF、关节 group、限位和运动学配置，接入 `humanoid_motion_control` 的完整 Motion/Action 链路。

## OpenArmX MuJoCo 端到端示例

工作区的 `openarmx_mujoco` 原本没有 ROS topic。新增的 `ros2_bridge.py` 故意以一个厂商驱动的形式暴露 OpenArmX 原生关节名和 topic；[openarmx_mujoco_driver.yaml](../config/openarmx_mujoco_driver.yaml) 则是平台的 mapping 配置。

先构建并 source 工作区：

```bash
source /opt/ros/humble/setup.bash
cd /home/czy/teleop_ws
colcon build --packages-up-to humanoid_driver_runtime
source install/setup.bash
```

终端 A 启动模拟厂商机器人。它是无界面的，使用现有的 MuJoCo runtime XML；若 XML 不存在，先运行一次 `control.py --check` 生成它。

```bash
source /opt/ros/humble/setup.bash
cd /home/czy/teleop_ws/src/openarmx_mujoco
python3 control.py --check
python3 ros2_bridge.py
```

终端 B 启动统一驱动节点。该节点加载真正的 pluginlib 驱动插件，不会直接访问 MuJoCo；其厂商侧只通过两个 OpenArmX topic 工作，平台侧固定为两个 HC topic。

```bash
source /opt/ros/humble/setup.bash
source /home/czy/teleop_ws/install/setup.bash
ros2 run humanoid_driver_runtime humanoid_driver_runtime_node --ros-args \
  --params-file /home/czy/teleop_ws/install/humanoid_driver_runtime/share/humanoid_driver_runtime/config/openarmx_mujoco_driver.yaml
```

预期终端 B 先短暂显示 `awaiting first valid ROS joint-state sample`，随后显示 `driver feedback is active`。终端 C 可以看到经过配置映射的平台逻辑反馈：

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /hc_teleop/joint_states
```

终端 D 以平台逻辑关节名发送一个低幅度目标；桥会映射为 OpenArmX 厂商关节命令：

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub -r 50 --qos-reliability reliable \
  /hc_teleop/joint_cmd sensor_msgs/msg/JointState \
  "{name: [openarmx_left_joint1], position: [0.2]}"
```

按 `Ctrl-C` 停止驱动节点时，`DriverRuntime` 会调用插件的 `stopAll()`，由插件发布当前测量位置作为保持命令。停止或暂停终端 A 后，驱动节点会在 250 ms 后把状态判为 stale，并忽略新的平台命令。
