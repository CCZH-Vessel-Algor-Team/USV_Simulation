cmake_minimum_required(VERSION 3.8)
project(radar_gz_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(marine_sensor_msgs REQUIRED)

# Gazebo Garden
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

# 创建可执行文件
add_executable(radar_gz_bridge src/radar_gz_bridge.cpp)

target_link_libraries(radar_gz_bridge
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

ament_target_dependencies(radar_gz_bridge
  rclcpp
  std_msgs
  builtin_interfaces
  marine_sensor_msgs
)

# 安装
install(TARGETS radar_gz_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
