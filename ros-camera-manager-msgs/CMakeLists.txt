cmake_minimum_required(VERSION 3.5)
project(ros_camera_manager_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(ros_camera_manager_msgs
  "./msg/CameraStream.msg"
  "./msg/CameraInfo.msg"
  "./srv/GetStatus.srv"
  "./srv/QueryCamera.srv"
  "./srv/RegisterCamera.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_package()
