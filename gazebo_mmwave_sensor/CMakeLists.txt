cmake_minimum_required(VERSION 3.8)
project(gazebo_mmwave_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ────────────────────────────────────────────────────────
# ROS 2 / ament dependencies
# ────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ────────────────────────────────────────────────────────
# Gazebo Harmonic (gz-sim8) via vendor packages
# ────────────────────────────────────────────────────────
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)

find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin2 REQUIRED)

find_package(gz_math_vendor REQUIRED)
find_package(gz-math7 REQUIRED)

find_package(gz_common_vendor REQUIRED)
find_package(gz-common5 REQUIRED)

# ────────────────────────────────────────────────────────
# Plugin shared library
# ────────────────────────────────────────────────────────
add_library(gazebo_mmwave_sensor SHARED
  src/MmwaveSensorPlugin.cpp
)

target_include_directories(gazebo_mmwave_sensor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(gazebo_mmwave_sensor
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-math7::gz-math7
  gz-common5::gz-common5
)

ament_target_dependencies(gazebo_mmwave_sensor
  rclcpp
  sensor_msgs
  geometry_msgs
)

# ────────────────────────────────────────────────────────
# Install
# ────────────────────────────────────────────────────────
install(TARGETS gazebo_mmwave_sensor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Install xacro/urdf examples so other packages can reference them
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../examples/urdf
  DESTINATION share/${PROJECT_NAME}/urdf
  OPTIONAL
)

ament_package()
