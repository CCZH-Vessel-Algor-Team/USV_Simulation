cmake_minimum_required(VERSION 3.8)
project(usv_mmwave_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(usv_interfaces REQUIRED)

find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)
find_package(gz-math7 REQUIRED)

add_library(usv_4d_radar_plugin SHARED
  src/4d_radar_plugin.cpp
)

add_library(usv_reciprocating_targets_plugin SHARED
  src/reciprocating_targets_plugin.cpp
)

target_include_directories(usv_4d_radar_plugin
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(usv_4d_radar_plugin
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(usv_4d_radar_plugin
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8
  gz-math7::gz-math7
)

target_link_libraries(usv_reciprocating_targets_plugin
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8
  gz-math7::gz-math7
)

add_executable(mmwave_4d_cloud_node src/mmwave_4d_cloud_node.cpp)
ament_target_dependencies(mmwave_4d_cloud_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(mmwave_cluster_node src/mmwave_cluster_node.cpp)
ament_target_dependencies(mmwave_cluster_node
  rclcpp
  sensor_msgs
  usv_interfaces
)

install(TARGETS usv_4d_radar_plugin
  usv_reciprocating_targets_plugin
  LIBRARY DESTINATION lib
)

install(TARGETS mmwave_4d_cloud_node
  mmwave_cluster_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
