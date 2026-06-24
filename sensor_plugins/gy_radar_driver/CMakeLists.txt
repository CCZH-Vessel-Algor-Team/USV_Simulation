cmake_minimum_required(VERSION 3.8)
project(gy_radar_driver)

# grid_map_ros（Humble）已链接系统 OpenCV 4.5d。若环境把 /usr/local 放进 CMAKE_PREFIX_PATH，
# 配置 grid_map_ros / cv_bridge 传递依赖时会 find_package(OpenCV) 到自编译 4.11，链接行出现
# /usr/local/lib/libopencv_*.so 与 libgrid_map_ros.so 混用，ld 报 “may conflict”。
# 对本包配置过程从 CMAKE_PREFIX_PATH 去掉 /usr/local（不动 SDK 的 NO_DEFAULT_PATH 查找）。
if(DEFINED CMAKE_PREFIX_PATH)
  set(_gy_prefix_filtered "")
  foreach(_p IN LISTS CMAKE_PREFIX_PATH)
    if(NOT _p STREQUAL "/usr/local")
      list(APPEND _gy_prefix_filtered "${_p}")
    endif()
  endforeach()
  set(CMAKE_PREFIX_PATH "${_gy_prefix_filtered}")
endif()

# 环境变量或旧缓存中的 OpenCV_DIR 常指向 /usr/local，导致链接 4.11；grid_map_ros 需与系统 4.5d 一致。
if(UNIX AND NOT APPLE AND EXISTS "/usr/lib/x86_64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake")
  set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4" CACHE PATH "Distro OpenCV for ROS Humble / grid_map" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ========================================
# 1. 查找依赖
# ========================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(marine_sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(usv_interfaces REQUIRED)
include_directories(${usv_interfaces_DIR}/../../../include)

# ========================================
# 2. SDK 配置
# ========================================
set(SDK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SDK_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)

include_directories(
  include 
  ${SDK_INCLUDE_DIR}
  ${SDK_INCLUDE_DIR}/gy_sdk
)

# 查找库文件
find_library(LIB_MARPA MarpaClient PATHS ${SDK_LIB_DIR} NO_DEFAULT_PATH)
find_library(LIB_IMAGE ImageClient PATHS ${SDK_LIB_DIR} NO_DEFAULT_PATH)
find_library(LIB_NAVI_UTILS NaviRadarUtils PATHS ${SDK_LIB_DIR} NO_DEFAULT_PATH)
find_library(LIB_NAVI_STORAGE NaviradarStorage PATHS ${SDK_LIB_DIR} NO_DEFAULT_PATH)
find_library(LIB_UDP UdpSocket PATHS ${SDK_LIB_DIR} NO_DEFAULT_PATH)
find_library(LIB_INI IniProfile PATHS ${SDK_LIB_DIR} NO_DEFAULT_PATH)
find_library(LIB_PPI PpiClient PATHS ${SDK_LIB_DIR} NO_DEFAULT_PATH)
find_library(LIB_PPI_CORE ppi PATHS ${SDK_LIB_DIR} NO_DEFAULT_PATH)

message(STATUS "Found MARPA Lib: ${LIB_MARPA}")
message(STATUS "Found Image Lib: ${LIB_IMAGE}")

if(NOT LIB_MARPA OR NOT LIB_IMAGE)
    message(FATAL_ERROR "关键库文件 (MarpaClient 或 ImageClient) 未找到！请检查 lib 目录。")
endif()

# ========================================
# 3. 雷达控制节点
# ========================================
add_executable(radar_control_node src/radar_control_node.cpp)

ament_target_dependencies(radar_control_node 
  rclcpp 
  std_msgs
  std_srvs
)

target_link_libraries(radar_control_node
  ${LIB_IMAGE} 
  ${LIB_PPI} 
  ${LIB_MARPA} 
  ${LIB_PPI_CORE} 
  ${LIB_UDP} 
  ${LIB_INI}
)

# ========================================
# 4. ARPA 目标节点 (使用 usv_interfaces)
# ========================================
add_executable(arpa_receiver_node src/arpa_receiver_node.cpp)

ament_target_dependencies(arpa_receiver_node
  rclcpp
  std_msgs
  std_srvs
  usv_interfaces  # 使用外部消息接口
)

# 链接 SDK 库
target_link_libraries(arpa_receiver_node
  ${LIB_MARPA}        
  ${LIB_IMAGE}        
  ${LIB_NAVI_UTILS}   
  ${LIB_NAVI_STORAGE}
  ${LIB_UDP}
  ${LIB_INI}
)

# ========================================
# 5. 雷达数据节点
# ========================================
add_executable(radar_data_node src/radar_data_node.cpp)

ament_target_dependencies(radar_data_node 
  rclcpp 
  sensor_msgs 
  std_msgs 
  marine_sensor_msgs
  usv_interfaces
)

target_link_libraries(radar_data_node
  ${LIB_IMAGE}
  ${LIB_NAVI_UTILS}
  ${LIB_UDP}
  ${LIB_INI}
)

# ========================================
# 6. 其他节点
# ========================================

# 转换节点
add_executable(radar_converter_node src/radar_converter_node.cpp)
ament_target_dependencies(radar_converter_node rclcpp sensor_msgs marine_sensor_msgs usv_interfaces)

# 建图节点
add_executable(radar_mapping_node src/radar_mapping_node.cpp)
ament_target_dependencies(radar_mapping_node rclcpp nav_msgs marine_sensor_msgs)

# NMEA 发送节点
add_executable(nmea_sender_node src/nmea_sender_node.cpp)
ament_target_dependencies(nmea_sender_node rclcpp nav_msgs nmea_msgs)

# 惯导接收节点
add_executable(radar_ins_node src/radar_ins_node.cpp)
ament_target_dependencies(radar_ins_node rclcpp)
target_link_libraries(radar_ins_node
  ${LIB_IMAGE} ${LIB_UDP} ${LIB_INI}
)

# ========================================
# 7. TF 发布节点
# ========================================
add_executable(radar_tf_node src/radar_tf_node.cpp)

ament_target_dependencies(radar_tf_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  usv_interfaces  # 如果 radar_tf_node 也使用了消息
)

# ========================================
# 8. 自适应栅格地图节点
# ========================================
add_executable(adaptive_radar_grid_map_node 
  src/adaptive_radar_grid_map_node.cpp)
ament_target_dependencies(adaptive_radar_grid_map_node
  rclcpp 
  nav_msgs 
  grid_map_core
  grid_map_ros
  grid_map_msgs 
  marine_sensor_msgs 
  tf2_ros 
  rcl_interfaces
  usv_interfaces
)

# ========================================
# 9. ARPA 距离调试节点 (使用 usv_interfaces)
# ========================================
add_executable(arpa_distance_debug src/arpa_distance_debug.cpp)

ament_target_dependencies(arpa_distance_debug
  rclcpp
  std_msgs
  std_srvs
  usv_interfaces  # 使用外部消息接口
)

# 链接 SDK 库
target_link_libraries(arpa_distance_debug
  ${LIB_MARPA}        
  ${LIB_IMAGE}        
  ${LIB_NAVI_UTILS}   
  ${LIB_NAVI_STORAGE}
  ${LIB_UDP}
  ${LIB_INI}
)

# ========================================
# 10. 安装规则
# ========================================
install(TARGETS 
  radar_control_node
  radar_data_node
  radar_converter_node
  radar_mapping_node
  nmea_sender_node
  arpa_receiver_node
  radar_ins_node
  radar_tf_node
  adaptive_radar_grid_map_node
  arpa_distance_debug
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY lib/ DESTINATION lib FILES_MATCHING PATTERN "*.so*")
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
