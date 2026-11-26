#pragma once

#include "ltr2/core.hpp"
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace ltr2_ros2 {

inline sensor_msgs::msg::LaserScan to_message(const ltr2::Scan& scan,
                                             const rclcpp::Time& received) {
    sensor_msgs::msg::LaserScan message;
    message.header.frame_id = "scan";
    // 完整帧接收时的系统时间；绝对时间对齐未验证，不使用核心的采集时间估计。
    message.header.stamp = received;
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
