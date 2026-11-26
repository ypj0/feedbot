#pragma once

#include "ltr2/core.hpp"
#include <cstdint>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace ltr2_ros2 {

inline sensor_msgs::msg::LaserScan to_message(const ltr2::Scan& scan,
                                             const rclcpp::Time& received) {
    sensor_msgs::msg::LaserScan message;
    message.header.frame_id = "scan";
    // First-beam estimate recovered from the device frame timing.
    message.header.stamp = rclcpp::Time(
        static_cast<int64_t>(scan.stamp * 1e9), RCL_SYSTEM_TIME);
    message.angle_min = scan.angle_min;
    message.angle_max = scan.angle_max;
    message.angle_increment = scan.angle_increment;
    message.scan_time = scan.scan_time;
    message.time_increment = scan.time_increment;
    message.range_min = 0.05;
    message.range_max = 40.0;
    message.ranges = scan.ranges;
    message.intensities = scan.intensities;
    return message;
}

}
