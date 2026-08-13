cmake_minimum_required(VERSION 3.8)
project(enc_grounding_warning_rviz)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(enc_grounding_warning_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)

set(panel_headers_to_moc
  include/enc_grounding_warning_rviz/grounding_warning_panel.hpp
)

set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
  src/grounding_warning_panel.cpp
  ${panel_headers_to_moc}
)

target_include_directories(${library_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${library_name}
  enc_grounding_warning_msgs
  pluginlib
  rclcpp
  rviz_common
  std_msgs
)

target_link_libraries(${library_name}
  rviz_common::rviz_common
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(
  TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_targets(${library_name} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  enc_grounding_warning_msgs
  pluginlib
  rclcpp
  rviz_common
  std_msgs
)
ament_package()
