cmake_minimum_required(VERSION 3.8)
project(mmwave_cloud_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# ── mmwave_cloud_colorizer executable ────────────────────────────────────────
add_executable(mmwave_cloud_colorizer
  src/mmwave_cloud_colorizer.cpp
)
ament_target_dependencies(mmwave_cloud_colorizer
  rclcpp
  sensor_msgs
)

install(TARGETS mmwave_cloud_colorizer
  DESTINATION lib/${PROJECT_NAME}
)

# ── Launch files ─────────────────────────────────────────────────────────────
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
