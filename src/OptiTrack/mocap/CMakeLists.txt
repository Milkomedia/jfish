# GPT의 도움으로 Pre_build없이 작동하는 cmake입니다... 수정 필수 (원상태로) 
# 8a278b2da263f31c4f6eae8a34d0675f03a1b680 commit point
# (단, 아래에 추가적인 joint cmd를 받는 부분은 추가해야함. 체크 해놓았다!!)

cmake_minimum_required(VERSION 3.10)
project(mocap)

# 컴파일 옵션
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# shared library rpath
set(CMAKE_INSTALL_RPATH "$ORIGIN")

# =========================
#   ROS2 / 라이브러리 의존성
# =========================
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(mocap_interfaces REQUIRED)
find_package(mujoco_interfaces REQUIRED)
find_package(watchdog_interfaces REQUIRED)
find_package(dynamixel_interfaces REQUIRED) # <========================= 여기 추가
find_package(imu_interfaces REQUIRED) # <========================= 여기 추가

# Eigen / PCL
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree search registration)

# Boost (thread, system)
find_package(Boost REQUIRED COMPONENTS thread system)
if(NOT Boost_FOUND)
  message(FATAL_ERROR "Could not find Boost.Thread/System")
endif()
message(STATUS "Boost include: ${Boost_INCLUDE_DIRS}")
message(STATUS "Boost libs:    ${Boost_LIBRARIES}")

# 공통 include 디렉토리
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# 필요하면 PCL 매크로 추가
add_definitions(${PCL_DEFINITIONS})

# =========================
#   executables (SIM 전용)
# =========================

# 실험/시뮬에서 사용하는 mocap worker 노드만 빌드
add_executable(mocap_worker src/mocap_worker.cpp)

ament_target_dependencies(mocap_worker
  rclcpp
  rclcpp_components
  tf2_ros
  sensor_msgs
  std_msgs
  geometry_msgs
  mocap_interfaces
  mujoco_interfaces
  watchdog_interfaces
  dynamixel_interfaces # <========================= 여기 추가
  imu_interfaces # <========================= 여기 추가
)

target_link_libraries(mocap_worker
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  pthread
)

# =========================
#   install & ament
# =========================

install(TARGETS
  mocap_worker
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
