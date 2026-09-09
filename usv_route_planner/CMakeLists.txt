cmake_minimum_required(VERSION 3.8)
project(usv_route_planner)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(usv_interfaces REQUIRED)

list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

add_library(route_planner_core
  src/grid_map.cpp
  src/route_planner.cpp
)
target_include_directories(route_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(route_planner_core geometry_msgs nav_msgs)
target_link_libraries(route_planner_core ${OpenCV_LIBS})

add_executable(route_planner_node src/route_planner_node.cpp)
target_include_directories(route_planner_node PRIVATE
  ${GeographicLib_INCLUDE_DIRS}
)
ament_target_dependencies(route_planner_node
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  usv_interfaces
)
target_link_libraries(route_planner_node
  route_planner_core
  ${GeographicLib_LIBRARIES}
)

install(TARGETS route_planner_core route_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_route_planner test/test_route_planner.cpp)
  target_link_libraries(test_route_planner route_planner_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(route_planner_core)
ament_export_dependencies(geometry_msgs nav_msgs)
ament_package()
