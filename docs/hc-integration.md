# HC 通用驱动与外接夹爪

三个通用包继续留在原 src，不依赖 humanoid_manager。`hc_group_position_driver.yaml` 是通用 ROS topic 驱动示例；必须按实际机器人更换关节、vendor 顺序、控制器话题与单位映射，不能将示例当作已配置的设备。逻辑臂单位 rad，vendor_to_logical_scales/offsets 承担映射。

`hc_safety_enabled: true` 接入 /control/emergency_stop、/control/hardware/enabled 和 /status/hardware/ready。硬件执行使能由 HC mux 管理，/control/teleop/enabled 仅启停 VR 输入，不能作为硬件停止条件；自定义 `output_enable_topic` 需同步更新。ready 由读到的新鲜真实反馈产生，不由进程启动产生；反馈丢失会撤销命令并锁定停止，恢复后需显式启用和新时间戳目标。既有配置缺省 false 保留旧入口。仅运行一种向标准臂反馈/ready 发布的硬件后端，不与 OpenArmX adapter 同时占用。

外接夹爪使用 humanoid_gripper 的 `hc_external_gripper.yaml` 示例，按侧配置 /action/right_hand/joint_states 或 left_hand，独立逻辑关节名、m/rad 单位和上下限。`require_feedback_stamp=true` 要求 JointState 源时间递增、新鲜；拒绝零时间、重放和未来时间。Float64 旧反馈入口保留，但只有接收时间，不能宣称知道硬件采样时间。HC 示例使用有源时间的 JointState。

通用驱动不提供统一固件采用命令证明。反馈、命令接收与实际执行分别解释；本地 stop/watchdog 仅调用插件提供的停止语义。各厂商插件必须实测该语义，不能仅凭 ROS 消息声称电机已停止。
