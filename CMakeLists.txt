cmake_minimum_required(VERSION 3.8)
project(ddsm115_ros2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(Boost REQUIRED CONFIG COMPONENTS headers)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages
set(msg_files
  "msg/Ddsm115Command.msg"
  "msg/Ddsm115Status.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# 1. Low-level driver library (no ROS dependencies, pure C++)
add_library(ddsm115_driver_client SHARED
  src/ddsm115_driver_client.cpp
)
target_include_directories(ddsm115_driver_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(ddsm115_driver_client PUBLIC cxx_std_20)
target_link_libraries(ddsm115_driver_client PUBLIC
  Boost::headers
)

# 2. Standalone ROS 2 node executable
add_executable(ddsm115_driver_node
  src/ddsm115_driver_node.cpp
  src/ddsm115_driver_node_main.cpp
)
target_include_directories(ddsm115_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(ddsm115_driver_node PUBLIC cxx_std_20)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(ddsm115_driver_node
  ddsm115_driver_client
  "${cpp_typesupport_target}"
)
ament_target_dependencies(ddsm115_driver_node
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  diagnostic_updater
)

# 3. ros2_control hardware interface plugin library
add_library(ddsm115_system_hardware SHARED
  src/ddsm115_system_hardware.cpp
)
target_include_directories(ddsm115_system_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(ddsm115_system_hardware PUBLIC cxx_std_20)
target_link_libraries(ddsm115_system_hardware
  ddsm115_driver_client
)
ament_target_dependencies(ddsm115_system_hardware
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

# Export plugin description file for ros2_control
pluginlib_export_plugin_description_file(hardware_interface ddsm115_hardware.xml)

# Install targets
install(TARGETS
  ddsm115_driver_client
  ddsm115_driver_node
  ddsm115_system_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/set_motor_id.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  ddsm115_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export interface dependencies
ament_export_include_directories(include)
ament_export_libraries(ddsm115_driver_client ddsm115_system_hardware)
ament_package()
