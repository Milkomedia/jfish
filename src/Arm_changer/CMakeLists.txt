cmake_minimum_required(VERSION 3.5)
project(arm_changer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sbus_interfaces REQUIRED)
find_package(allocator_interfaces REQUIRED)
find_package(dynamixel_interfaces REQUIRED) 
find_package(watchdog_interfaces REQUIRED)

add_executable(arm_changer src/arm_changer.cpp)

target_include_directories(arm_changer PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(arm_changer
  rclcpp
  sbus_interfaces
  dynamixel_interfaces
  allocator_interfaces
  watchdog_interfaces
  Eigen3
)

install(TARGETS
  arm_changer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

ament_package()