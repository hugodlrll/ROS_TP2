cmake_minimum_required(VERSION 3.5)
project(handball_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# Do not put find_package(rclcpp REQUIRED) in a pure message folder
# especially if it defines services and/or actions.
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages, services and actions with type support
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BallPosition.msg"
  "msg/BallState.msg"
  "msg/BallVelocity.msg"
  "msg/CoachInstruction.msg"
  "msg/PlanarTwist.msg"
  "msg/PlayerControl.msg"
  "msg/PlayerState.msg"
  "msg/Posture.msg"
  "msg/Whistle.msg"
  "srv/SubstitutionApproval.srv"
  "action/SubstitutionExecution.action"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# This line must be after rosidl_generate_interfaces
ament_package()

