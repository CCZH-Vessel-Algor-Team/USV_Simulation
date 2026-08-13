cmake_minimum_required(VERSION 3.8)
project(enc_grounding_warning_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DepthGrid.msg"
  "msg/UKCState.msg"
  "msg/GroundingAlert.msg"
  "msg/GroundingRiskGrid.msg"
  "msg/RouteDepthProfile.msg"
  "srv/RouteCheck.srv"
  DEPENDENCIES std_msgs nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
