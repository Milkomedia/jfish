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
find_package(dynamixel_interfaces REQUIRED)

# Eigen / PCL
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree search registration)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

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
  dynamixel_interfaces
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