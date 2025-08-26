cmake_minimum_required(VERSION 3.10)
project(controller_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

set(msg_files
  "msg/ControllerOutput.msg"
  "msg/ControllerDebugVal.msg"
  "msg/EstimatorOutput.msg"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp/
  DESTINATION include
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_package()