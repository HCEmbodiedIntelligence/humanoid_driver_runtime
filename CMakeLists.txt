cmake_minimum_required(VERSION 3.8)
project(humanoid_driver_runtime)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(humanoid_driver_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(humanoid_driver_runtime_core SHARED
  src/diagnostics_runtime.cpp
  src/driver_runtime.cpp)
target_include_directories(humanoid_driver_runtime_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(humanoid_driver_runtime_core PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(humanoid_driver_runtime_core
  diagnostic_msgs
  humanoid_driver_interface
  pluginlib
  rclcpp)

add_library(humanoid_mock_driver SHARED src/mock_robot_driver.cpp)
target_include_directories(humanoid_mock_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(humanoid_mock_driver PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(humanoid_mock_driver humanoid_driver_interface pluginlib)

add_library(humanoid_ros_topic_driver SHARED src/ros_topic_robot_driver.cpp)
target_include_directories(humanoid_ros_topic_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(humanoid_ros_topic_driver PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(humanoid_ros_topic_driver
  humanoid_driver_interface
  pluginlib
  rclcpp
  sensor_msgs)

pluginlib_export_plugin_description_file(
  humanoid_driver_interface plugins/driver_plugins.xml)

add_executable(humanoid_driver_runtime_node src/humanoid_driver_runtime_node.cpp)
target_compile_options(humanoid_driver_runtime_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(humanoid_driver_runtime_node humanoid_driver_runtime_core)
ament_target_dependencies(humanoid_driver_runtime_node
  diagnostic_msgs
  humanoid_driver_interface
  rclcpp
  sensor_msgs)

install(TARGETS
  humanoid_driver_runtime_core
  humanoid_mock_driver
  humanoid_ros_topic_driver
  humanoid_driver_runtime_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config docs launch plugins DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_driver_runtime test/test_driver_runtime.cpp)
  target_link_libraries(test_driver_runtime humanoid_driver_runtime_core)

  ament_add_gtest(test_mock_lifecycle test/test_mock_lifecycle.cpp)
  target_link_libraries(test_mock_lifecycle humanoid_mock_driver)
  ament_target_dependencies(test_mock_lifecycle humanoid_driver_interface)

  ament_add_gtest(test_mock_closed_loop test/test_mock_closed_loop.cpp)
  target_link_libraries(test_mock_closed_loop humanoid_mock_driver)
  ament_target_dependencies(test_mock_closed_loop humanoid_driver_interface)

  ament_add_gtest(test_plugin_loading test/test_plugin_loading.cpp)
  ament_target_dependencies(test_plugin_loading humanoid_driver_interface pluginlib)

  ament_add_gtest(test_ros_topic_robot_driver test/test_ros_topic_robot_driver.cpp)
  target_link_libraries(test_ros_topic_robot_driver humanoid_ros_topic_driver)
  ament_target_dependencies(test_ros_topic_robot_driver
    humanoid_driver_interface
    rclcpp
    sensor_msgs)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  diagnostic_msgs
  humanoid_driver_interface
  pluginlib
  rclcpp
  sensor_msgs)

ament_package()
