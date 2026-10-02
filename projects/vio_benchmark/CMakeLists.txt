cmake_minimum_required(VERSION 3.16)
project(vio_benchmark)

# ─────────────────────────────────────────────────────────────────────────────
# Flies a helix round tower_1, or a transit down the line, so a visual-inertial
# estimator (OpenVINS) can be scored on each. Whole trial from the HOST:
#
#     ./projects/vio_benchmark/scripts/vio_trial.sh helix
#
# From walker's mission list (key p) it flies the default profile, helix:
# walker passes no ROS parameters. By hand, inside the container:
#
#     ros2 run vio_benchmark vio_benchmark --ros-args -p profile:=transit
#
# See README.md beside this file.
# ─────────────────────────────────────────────────────────────────────────────

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(psdk_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(vio_benchmark src/main.cpp)
ament_target_dependencies(vio_benchmark
  rclcpp geometry_msgs nav_msgs psdk_interfaces sensor_msgs std_msgs std_srvs)

install(TARGETS vio_benchmark
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
