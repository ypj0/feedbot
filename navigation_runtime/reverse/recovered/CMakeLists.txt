cmake_minimum_required(VERSION 3.16)
project(feedbot_recovered_models LANGUAGES C)
# Pure offline models. This target has no device or networking implementation.
add_library(feedbot_recovered_models SHARED
  charge_core.c charge_execute.c driver_smoothing.c driver_frame.c dwa_pid.c
  lidar_packet.c sub_core.c sub_target.c sub_kinematics.c motor_output.c)
target_compile_features(feedbot_recovered_models PRIVATE c_std_11)
if(CMAKE_C_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(feedbot_recovered_models PRIVATE -Wall -Wextra -Werror -ffp-contract=off)
endif()
if(UNIX AND NOT APPLE)
  target_link_libraries(feedbot_recovered_models PRIVATE m)
endif()
