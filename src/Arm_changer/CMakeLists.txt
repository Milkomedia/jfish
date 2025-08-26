cmake_minimum_required(VERSION 3.5)
project(arm_changer)

# 기본 의존성
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

# 인터페이스 패키지들 (메시지/서비스 헤더 사용)
find_package(controller_interfaces REQUIRED)
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
  Eigen3
  controller_interfaces
  sbus_interfaces
  allocator_interfaces
  dynamixel_interfaces
  watchdog_interfaces
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
