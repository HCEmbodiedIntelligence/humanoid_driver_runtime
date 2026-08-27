# 添加独立厂商驱动插件

只有厂商接口不能由现成 ROS Topic 适配器表达时才需要写驱动。驱动放在独立 ROS 2 包；
`humanoid_driver_runtime` 只按配置加载它，不需要增加机器人类型、工厂分支或编译依赖。

## 插件类

类继承 `humanoid_driver_interface::RobotDriverPlugin`，并实现配置、连接、激活、状态读取、关节命令、
停止和健康状态接口。插件边界统一使用 SI 单位；厂商单位和坐标变换在插件内部或 YAML mapping 中完成。

```cpp
#include <pluginlib/class_list_macros.hpp>
#include <humanoid_driver_interface/robot_driver_plugin.hpp>

#include "my_robot_driver/my_robot_driver.hpp"

PLUGINLIB_EXPORT_CLASS(
  my_robot_driver::MyRobotDriver,
  humanoid_driver_interface::RobotDriverPlugin)
```

`stopAll()` 是安全路径，必须幂等，并在返回成功前执行保持当前位置或设备等价的安全停止。

## pluginlib 导出

`plugins/driver_plugins.xml`：

```xml
<library path="my_robot_driver">
  <class
    name="my_robot_driver/MyRobotDriver"
    type="my_robot_driver::MyRobotDriver"
    base_class_type="humanoid_driver_interface::RobotDriverPlugin">
    <description>My robot vendor driver.</description>
  </class>
</library>
```

`CMakeLists.txt` 的关键部分：

```cmake
find_package(humanoid_driver_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(my_robot_driver SHARED src/my_robot_driver.cpp)
ament_target_dependencies(my_robot_driver
  humanoid_driver_interface pluginlib)

pluginlib_export_plugin_description_file(
  humanoid_driver_interface plugins/driver_plugins.xml)

install(TARGETS my_robot_driver LIBRARY DESTINATION lib)
```

`package.xml` 需要声明依赖和插件描述：

```xml
<depend>humanoid_driver_interface</depend>
<depend>pluginlib</depend>
<export>
  <build_type>ament_cmake</build_type>
  <humanoid_driver_interface plugin="${prefix}/plugins/driver_plugins.xml"/>
</export>
```

## 由配置选择插件

机器人 profile 引用的 driver YAML 选择导出的类：

```yaml
humanoid_driver_runtime:
  ros__parameters:
    plugin_class: my_robot_driver/MyRobotDriver
    command_watchdog_ms: 100.0
    control_frequency_hz: 100.0
    platform_joint_state_topic: /hc_teleop/joint_states
    platform_joint_command_topic: /hc_teleop/joint_cmd
    joint_names: [joint1, joint2]
    vendor_joint_names: [axis_1, axis_2]
    vendor_joint_groups: [arm, arm]
    vendor_to_logical_scales: [1.0, 1.0]
    vendor_to_logical_offsets_rad: [0.0, 0.0]
    plugin_parameters:
      - endpoint=192.168.1.10
```

构建并 source 插件包后，通用 runtime 会从 pluginlib 索引发现这个类。新增或切换厂商驱动只改
profile/YAML 的包资源和 `plugin_class`，不修改 `humanoid_driver_runtime` 或运动服务源码。
