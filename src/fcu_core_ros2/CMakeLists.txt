cmake_minimum_required(VERSION 3.8)
project(fcu_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(quadrotor_msgs REQUIRED)

add_executable(fcu_bridge_001 src/fcu_bridge_001.cpp)
add_executable(fcu_bridge_002 src/fcu_bridge_002.cpp)
add_executable(fcu_bridge_003 src/fcu_bridge_003.cpp)
add_executable(fcu_bridge_004 src/fcu_bridge_004.cpp)
add_executable(fcu_bridge_005 src/fcu_bridge_005.cpp)
add_executable(fcu_bridge_006 src/fcu_bridge_006.cpp)
add_executable(fcu_command src/fcu_command.cpp)
add_executable(fcu_mission src/fcu_mission.cpp)

ament_target_dependencies(
  fcu_bridge_001
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "quadrotor_msgs"
)

install(TARGETS fcu_bridge_001
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(
  fcu_bridge_002
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "quadrotor_msgs"
)

install(TARGETS fcu_bridge_002
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(
  fcu_bridge_003
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "quadrotor_msgs"
)

install(TARGETS fcu_bridge_003
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(
  fcu_bridge_004
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "quadrotor_msgs"
)

install(TARGETS fcu_bridge_004
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(
  fcu_bridge_005
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "quadrotor_msgs"
)

install(TARGETS fcu_bridge_005
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(
  fcu_bridge_006
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "quadrotor_msgs"
)

install(TARGETS fcu_bridge_006
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(
  fcu_command
  "rclcpp"
  "std_msgs"
)

install(TARGETS fcu_command
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(
  fcu_mission
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "quadrotor_msgs"
)

install(TARGETS fcu_mission
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
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
