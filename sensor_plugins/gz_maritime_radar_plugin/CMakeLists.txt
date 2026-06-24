cmake_minimum_required(VERSION 3.10)
project(gz_maritime_radar_plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖
find_package(gz-sim8 REQUIRED)          # Gazebo Garden
find_package(gz-plugin2 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-common5 REQUIRED)

# 创建插件库
add_library(MaritimeRadarPlugin SHARED
  src/MaritimeRadarPlugin.cc
)

target_include_directories(MaritimeRadarPlugin PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(MaritimeRadarPlugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
  gz-common5::gz-common5
)

# 安装
install(TARGETS MaritimeRadarPlugin
  LIBRARY DESTINATION lib
)

# 可选: 安装模型文件
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
  OPTIONAL
)
