cmake_minimum_required(VERSION 3.8)
project(virtual_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) 

# C++ Node
# add_executable(lidar_odom_cpp src/lidar_odom_node.cpp)
# target_include_directories(lidar_odom_cpp PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
#   ${PCL_INCLUDE_DIRS})
# ament_target_dependencies(lidar_odom_cpp
#   rclcpp
#   sensor_msgs
#   nav_msgs
#   std_msgs
#   pcl_conversions
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
#   geometry_msgs
# )
# target_link_libraries(lidar_odom_cpp ${PCL_LIBRARIES})

# install(TARGETS lidar_odom_cpp
#   DESTINATION lib/${PROJECT_NAME})

# Python Node - Install scripts

install(PROGRAMS
  scripts/lidar_odom_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
