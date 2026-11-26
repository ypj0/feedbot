# 3.10 是车机宿主机（Ubuntu 18.04）的 cmake 版本，保持兼容以便直接在车机上编译验证。
cmake_minimum_required(VERSION 3.10)
project(feedbot_motion_authority_core LANGUAGES CXX)

# 纯 C++17 核心库（不依赖 ROS）。ROS 2 节点封装在后续切片中单独加入。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(FEEDBOT_SANITIZE "Build tests with ASan/UBSan" ON)

add_library(motion_authority_core INTERFACE)
target_include_directories(motion_authority_core INTERFACE include)

enable_testing()
add_executable(motion_authority_core_test tests/core_test.cpp)
target_link_libraries(motion_authority_core_test PRIVATE motion_authority_core)
target_compile_options(motion_authority_core_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
if(FEEDBOT_SANITIZE)
  target_compile_options(motion_authority_core_test PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=undefined)
  target_link_libraries(motion_authority_core_test PRIVATE -fsanitize=address,undefined)
endif()
add_test(NAME motion_authority_core COMMAND motion_authority_core_test)
