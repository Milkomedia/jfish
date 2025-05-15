cmake_minimum_required(VERSION 3.10)
project(imu_worker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(imu_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mujoco_interfaces REQUIRED)
find_package(watchdog_interfaces REQUIRED)

# Include directories
include_directories(include)

# Add executables
add_executable(imu_worker src/imu_worker.cpp)

# Link libraries
ament_target_dependencies(imu_worker
  rclcpp
  rclcpp_components
  imu_interfaces
  sensor_msgs
  mujoco_interfaces
  watchdog_interfaces
)

install(TARGETS
  imu_worker
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

# Export dependencies
ament_package()