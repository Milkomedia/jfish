cmake_minimum_required(VERSION 3.10)
project(controller_geom)
add_compile_options(-Wno-class-memaccess)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interfaces REQUIRED)
find_package(sbus_interfaces REQUIRED)
find_package(imu_interfaces REQUIRED)
find_package(dynamixel_interfaces REQUIRED)
find_package(mujoco_interfaces REQUIRED)
find_package(mocap_interfaces REQUIRED)
find_package(watchdog_interfaces REQUIRED)

# Include directories
include_directories(include
  ${EIGEN3_INCLUDE_DIR}
)

# Build the controller library from the core source files.
add_library(controller
  src/fdcl_control.cpp
  src/fdcl_matrix_utils.cpp
)
target_link_libraries(controller PUBLIC)

add_executable(controller_worker_geom src/controller_worker.cpp)
target_link_libraries(controller_worker_geom controller)
ament_target_dependencies(controller_worker_geom
  Eigen3
  rclcpp
  rclcpp_components
  controller_interfaces
  sbus_interfaces
  imu_interfaces
  dynamixel_interfaces
  mujoco_interfaces
  mocap_interfaces
  watchdog_interfaces
)

install(TARGETS
  controller_worker_geom
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# Export dependencies
ament_package()