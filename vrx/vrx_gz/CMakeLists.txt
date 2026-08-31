cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(vrx_gz)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
if(COMPILER_SUPPORTS_CXX17)
    add_compile_options(-std=c++17)
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif()

find_package(ament_cmake REQUIRED)

find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

find_package(gz-common5 REQUIRED COMPONENTS graphics)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
find_package(gz-fuel_tools9 REQUIRED)
find_package(gz-math7 REQUIRED)
set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})
find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
find_package(gz-rendering8 REQUIRED)
set(GZ_RENDERING_VER ${gz-rendering8_VERSION_MAJOR})
find_package(gz-sensors8 REQUIRED)
set(GZ_SENSORS_VER ${gz-sensors8_VERSION_MAJOR})
find_package(gz-utils2 REQUIRED)
set(GZ_UTILS_VER ${gz-utils2_VERSION_MAJOR})
find_package(sdformat14 REQUIRED)

find_package(std_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

#============================================================================
# Hooks
ament_environment_hooks("hooks/resource_paths.dsv.in")
ament_environment_hooks("hooks/resource_paths.sh")

#============================================================================

# Waves
add_library(Waves SHARED
  src/Wavefield.cc
)
target_link_libraries(Waves PUBLIC
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-sim${GZ_SIM_VER}::core
  gz-math${GZ_MATH_VER}
  Eigen3::Eigen
)

# Buoyancy
add_library(PolyhedraBuoyancyDrag SHARED
  src/PolyhedraBuoyancyDrag.cc
  src/PolyhedronVolume.cc
  src/ShapeVolume.cc
)
target_link_libraries(PolyhedraBuoyancyDrag PUBLIC
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-sim${GZ_SIM_VER}::core
  gz-math${GZ_MATH_VER}
  Waves
)
# Default USV simulation runtime plugins. Competition scoring and official VRX
# launch support are intentionally excluded from this lean resource package.
list(APPEND VRX_GZ_PLUGINS
  PublisherPlugin
  Surface
  TargetShipController
  USVWind
  WaveVisual
)

foreach(PLUGIN ${VRX_GZ_PLUGINS})
  add_library(${PLUGIN} SHARED src/${PLUGIN}.cc)
  target_link_libraries(${PLUGIN} PUBLIC
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-math${GZ_MATH_VER}
    gz-sim${GZ_SIM_VER}::core
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
    gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
    gz-utils${GZ_UTILS_VER}::gz-utils${GZ_UTILS_VER}
    gz-msgs10::gz-msgs10
    gz-transport13::gz-transport13
    Waves
    Eigen3::Eigen
  )
endforeach()

install(
  TARGETS Waves PolyhedraBuoyancyDrag ${VRX_GZ_PLUGINS}
  DESTINATION lib)

#============================================================================
# Resources
install(DIRECTORY
  models
  worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
