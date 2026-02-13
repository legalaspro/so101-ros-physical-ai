cmake_minimum_required(VERSION 3.8)
project(episode_recorder)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

# Episode Recorder Node 
add_executable(episode_recorder_node
  src/episode_recorder_node.cpp
  src/episode_recorder.cpp
)
ament_target_dependencies(episode_recorder_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  rosbag2_cpp
  rosbag2_storage
  std_srvs
)

# Teleop Episode Keyboard 
add_executable(teleop_episode_keyboard
  src/teleop_episode_keyboard.cpp
)
ament_target_dependencies(teleop_episode_keyboard
  rclcpp
  std_srvs
)

# Install Targets
install(TARGETS
  episode_recorder_node
  teleop_episode_keyboard
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# Install share files, launch config
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
