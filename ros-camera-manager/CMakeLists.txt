cmake_minimum_required(VERSION 3.5)
project(ros_camera_manager)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros_camera_manager_msgs REQUIRED)

add_library(ros_camera_manager ./src/client.cc)
target_include_directories(ros_camera_manager PRIVATE include)
ament_target_dependencies(ros_camera_manager 
  rclcpp 
  ros_camera_manager_msgs
)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(ros_camera_manager HAS_LIBRARY_TARGET)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  TARGETS ros_camera_manager
  EXPORT ros_camera_manager
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_python_install_package(ros_camera_manager)

install(PROGRAMS
  ./ros_camera_manager/server.py
  ./ros_camera_manager/cli.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()