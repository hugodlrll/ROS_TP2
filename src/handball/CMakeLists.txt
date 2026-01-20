cmake_minimum_required(VERSION 3.5)
project(handball)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find required packages
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(handball_msgs REQUIRED)

# Include headers (if you create include/ directory)
# include_directories(include)

# === Executables ===

add_executable(coach src/coach.cpp)
add_executable(referee src/referee_amaury.cpp)
ament_target_dependencies(coach
  rclcpp
  handball_msgs
)

ament_target_dependencies(referee
  rclcpp
  rclcpp_action
  handball_msgs
)

# === Installation ===
install(
  TARGETS coach
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS referee
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# === Testing (optional) ===
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()

