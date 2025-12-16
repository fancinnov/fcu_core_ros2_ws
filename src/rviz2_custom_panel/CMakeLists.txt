cmake_minimum_required(VERSION 3.8)
project(rviz2_custom_panel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

qt5_wrap_cpp(MOC_FILES
  src/mypanel.h
)

set(SRC_FILES
  src/mypanel.cpp
)

add_library(mypanel SHARED ${MOC_FILES} ${SRC_FILES})

target_link_libraries(
  mypanel
  Qt5::Widgets
)

ament_target_dependencies(
  mypanel
  rviz_common
  rclcpp
  std_msgs
)

install(
  TARGETS mypanel
  DESTINATION lib/${PROJECT_NAME}
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

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_package()
