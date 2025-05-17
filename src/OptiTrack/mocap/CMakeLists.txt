cmake_minimum_required(VERSION 3.10)
project(mocap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set rpath for shared libraries
set(CMAKE_INSTALL_RPATH "$ORIGIN")

# where the prebuilts live
set(OPTI_ROOT "/home/strider/Pre_build/deps_opti")

# Find ROS2 deps
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

# For Eigen3 and PCL
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io kdtree search registration)

find_package(Boost REQUIRED COMPONENTS thread system)
if(NOT Boost_FOUND)
  message(FATAL_ERROR "Could not find Boost.Thread/System")
endif()
message(STATUS "Boost include: ${Boost_INCLUDE_DIRS}")
message(STATUS "Boost libs:    ${Boost_LIBRARIES}")

# ----------------------------------------------------------------------------
# 1) librigidbodytracker headers & library
# ----------------------------------------------------------------------------

find_path(RBT_INCLUDE_DIR
  NAMES librigidbodytracker/rigid_body_tracker.h
  PATHS "${OPTI_ROOT}/librigidbodytracker/include"
  NO_DEFAULT_PATH
)
find_library(RBT_LIB
  NAMES librigidbodytracker
  PATHS "${OPTI_ROOT}/build/librigidbodytracker"
  NO_DEFAULT_PATH
)
if(NOT RBT_INCLUDE_DIR OR NOT RBT_LIB)
  message(FATAL_ERROR "Could not find librigidbodytracker in ${OPTI_ROOT}")
endif()
message(STATUS "RBT_INCLUDE_DIR = ${RBT_INCLUDE_DIR}")
message(STATUS "RBT_LIB         = ${RBT_LIB}")

# ----------------------------------------------------------------------------
# 2) libmotioncapture headers & library
# ----------------------------------------------------------------------------

find_path(MC_INCLUDE_DIR
  NAMES libmotioncapture/motioncapture.h
  PATHS "${OPTI_ROOT}/libmotioncapture/include"
  NO_DEFAULT_PATH
)
find_library(MC_LIB
  NAMES libmotioncapture
  PATHS "${OPTI_ROOT}/build/libmotioncapture"
  NO_DEFAULT_PATH
)
if(NOT MC_INCLUDE_DIR OR NOT MC_LIB)
  message(FATAL_ERROR "Could not find libmotioncapture in ${OPTI_ROOT}")
endif()
message(STATUS "MC_INCLUDE_DIR  = ${MC_INCLUDE_DIR}")
message(STATUS "MC_LIB          = ${MC_LIB}")

# ----------------------------------------------------------------------------
# 3) common include directories
# ----------------------------------------------------------------------------

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# ----------------------------------------------------------------------------
# NatNet SDK (OptiTrack) integration
# ----------------------------------------------------------------------------

# 1) find NatNet include dir
find_path(NATNET_INCLUDE_DIR
  NAMES NatNetClient.h
  PATHS "${OPTI_ROOT}/libmotioncapture/deps/NatNetSDKCrossplatform/include"
  NO_DEFAULT_PATH
)

# 2) find NatNet library
find_library(NATNET_LIB
  NAMES NatNet
  PATHS "${OPTI_ROOT}/libmotioncapture/deps/NatNetSDKCrossplatform/lib/ubuntu"
  NO_DEFAULT_PATH
)

if(NOT NATNET_INCLUDE_DIR OR NOT NATNET_LIB)
  message(FATAL_ERROR "Could not find NatNet SDK in ${OPTI_ROOT}/libmotioncapture/deps/NatNetSDKCrossplatform")
endif()

message(STATUS "Found NatNet include: ${NATNET_INCLUDE_DIR}")
message(STATUS "Found NatNet lib:     ${NATNET_LIB}")

# ----------------------------------------------------------------------------
# Vicon DataStream SDK integration
# ----------------------------------------------------------------------------

# SDK 최상위
set(VICON_SDK_ROOT "${OPTI_ROOT}/libmotioncapture/deps/vicon-datastream-sdk")
# CMake 빌드 결과물이 있는 디렉터리
set(VICON_SDK_BUILD "${VICON_SDK_ROOT}/build")

# 1) 헤더 위치: 소스 트리의 CPP 래퍼 폴더
find_path(VICON_INCLUDE_DIR
  NAMES DataStreamClient.h
  PATHS
    "${VICON_SDK_ROOT}/Vicon/CrossMarket/DataStream/ViconDataStreamSDK_CPP"
  NO_DEFAULT_PATH
)

# 2) 라이브러리 위치: build 디렉터리에 생성된 정적 라이브러리(.a)
find_library(VICON_LIB
  NAMES ViconDataStreamSDK_CPP
  PATHS
    "${VICON_SDK_BUILD}"
  NO_DEFAULT_PATH
)

if(NOT VICON_INCLUDE_DIR OR NOT VICON_LIB)
  message(FATAL_ERROR
    "Could not find Vicon DataStream SDK.\n"
    "Headers under: ${VICON_SDK_ROOT}/Vicon/CrossMarket/DataStream/ViconDataStreamSDK_CPP\n"
    "Library under: ${VICON_SDK_BUILD}")
endif()

message(STATUS "VICON_INCLUDE_DIR = ${VICON_INCLUDE_DIR}")
message(STATUS "VICON_LIB         = ${VICON_LIB}")

# ----------------------------------------------------------------------------
# Qualisys QTM SDK integration
# ----------------------------------------------------------------------------

# SDK 소스 루트와 빌드 결과물이 있는 경로
set(QTM_SDK_ROOT "${OPTI_ROOT}/libmotioncapture/deps/qualisys_cpp_sdk")
set(QTM_SDK_BUILD "${QTM_SDK_ROOT}/build")

# 1) 헤더 위치: RTProtocol.h 가 있는 곳
find_path(QTM_INCLUDE_DIR
  NAMES RTProtocol.h
  PATHS "${QTM_SDK_ROOT}"
  NO_DEFAULT_PATH
)

# 2) 라이브러리 위치: build 디렉터리에 생성된 .a 파일
find_library(QTM_LIB
  NAMES CRTProtocol qualisys_cpp_sdk
  PATHS "${QTM_SDK_BUILD}"
  NO_DEFAULT_PATH
)

if(NOT QTM_INCLUDE_DIR OR NOT QTM_LIB)
  message(FATAL_ERROR
    "Could not find Qualisys CRTProtocol SDK.\n"
    "Headers under: ${QTM_SDK_ROOT}\n"
    "Library under: ${QTM_SDK_BUILD}")
endif()

message(STATUS "QTM_INCLUDE_DIR = ${QTM_INCLUDE_DIR}")
message(STATUS "QTM_LIB         = ${QTM_LIB}")

# ----------------------------------------------------------------------------
# 4) motion_capture_tracking_node
# ----------------------------------------------------------------------------

add_executable(motion_capture_tracking_node src/motion_capture_tracking_node.cpp)
ament_target_dependencies(motion_capture_tracking_node
  rclcpp tf2_ros sensor_msgs
  std_msgs geometry_msgs
  mocap_interfaces
)
# attach external headers
target_include_directories(motion_capture_tracking_node PUBLIC
  ${MC_INCLUDE_DIR}
  ${RBT_INCLUDE_DIR}
  ${NATNET_INCLUDE_DIR}
  ${VICON_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${QTM_INCLUDE_DIR}
)
# attach external libs plus PCL modules
target_link_libraries(motion_capture_tracking_node
  ${MC_LIB}
  ${NATNET_LIB}
  ${VICON_LIB}
  ${QTM_LIB}
  ${RBT_LIB}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  pthread
)

# ----------------------------------------------------------------------------
# 5) mocap_worker
# ----------------------------------------------------------------------------

add_executable(mocap_worker src/mocap_worker.cpp)
ament_target_dependencies(mocap_worker
  rclcpp rclcpp_components tf2_ros sensor_msgs
  std_msgs geometry_msgs
  mocap_interfaces mujoco_interfaces watchdog_interfaces
)
target_include_directories(mocap_worker PUBLIC
  ${MC_INCLUDE_DIR}
  ${RBT_INCLUDE_DIR}
  ${NATNET_INCLUDE_DIR}
  ${NATNET_INCLUDE_DIR}
  ${VICON_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)
target_link_libraries(mocap_worker
  ${MC_LIB}
  ${NATNET_LIB}
  ${QTM_LIB}
  ${RBT_LIB}
  ${RBT_LIB}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  pthread
)

# ----------------------------------------------------------------------------
# 6) install & ament
# ----------------------------------------------------------------------------

install(TARGETS
  mocap_worker
  motion_capture_tracking_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY config  DESTINATION share/${PROJECT_NAME}/)

if ((CMAKE_SYSTEM_NAME STREQUAL "Linux") AND (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64"))
  install(FILES 
    "${OPTI_ROOT}/libmotioncapture/deps/NatNetSDKCrossplatform/lib/ubuntu/libNatNet.so"
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_package()
