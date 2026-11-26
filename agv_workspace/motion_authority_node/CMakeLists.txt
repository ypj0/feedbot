cmake_minimum_required(VERSION 3.10)
project(feedbot_motion_authority_node)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(feedbot_motion_interfaces REQUIRED)

# motion_authority 是纯 C++ 头文件库
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../motion_authority/include)

add_executable(motion_authority_node src/node.cpp)
ament_target_dependencies(motion_authority_node
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  feedbot_motion_interfaces
)
target_compile_options(motion_authority_node PRIVATE -Wall -Wextra -Wpedantic -Werror)

install(TARGETS motion_authority_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
