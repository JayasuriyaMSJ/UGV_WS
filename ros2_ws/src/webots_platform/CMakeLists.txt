cmake_minimum_required(VERSION 3.8)
project(webots_platform)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(webots_ros2_driver REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)


add_library(
  ${PROJECT_NAME}
  SHARED
  src/UGV_Driver.cpp
  # src/Rosbot_Driver.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

pluginlib_export_plugin_description_file(
  webots_ros2_driver
  ugv_driver.xml
)

ament_target_dependencies(
  ${PROJECT_NAME}
  pluginlib
  rclcpp
  webots_ros2_driver
  nav_msgs
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install additional directories.
install(DIRECTORY
  launch
  resources
  worlds
  config
  rviz
  controllers
  protos
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)

install(PROGRAMS
  scripts/ugv_controller.py
  scripts/spawn_proto_robot.py
  scripts/odom_debugger.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
