cmake_minimum_required(VERSION 3.8)
project(so101_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# Add executables 
add_executable(teleop_split src/teleop_split.cpp)
add_executable(teleop src/teleop.cpp)

# Link dependencies
ament_target_dependencies(teleop
	rclcpp
	std_msgs
  sensor_msgs
  trajectory_msgs)
ament_target_dependencies(teleop_split
	rclcpp
  rclcpp_action
	std_msgs
  sensor_msgs
  trajectory_msgs
  control_msgs)

# Install executables
install(TARGETS 
  teleop
	teleop_split
	DESTINATION lib/${PROJECT_NAME})

# Install share files, launch 
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
