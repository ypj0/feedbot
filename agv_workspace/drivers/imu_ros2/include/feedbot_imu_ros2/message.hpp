#pragma once
#include "wit_decoder.hpp"
#include <optional>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace feedbot_imu_ros2 {
struct ImuSample {
    std::array<double, 3> angular_velocity{};
    std::array<double, 3> linear_acceleration{};
};

class GyroStream {
    feedbot_imu::Decoder decoder_;
    double last_gyro_ = 0.0;

public:
    std::optional<ImuSample> push(uint8_t byte, double receipt) {
        const auto before = decoder_.accepted;
        decoder_.push(byte, receipt);
        if (decoder_.accepted == before) return std::nullopt;
        const auto sample = decoder_.snapshot(receipt, 0.2, 0.1);
        if (sample.received[1] <= last_gyro_) return std::nullopt;
        last_gyro_ = sample.received[1];
        return ImuSample{sample.angular_velocity, sample.acceleration};
    }

    uint64_t accepted() const { return decoder_.accepted; }
    uint64_t rejected() const { return decoder_.rejected; }
};

inline sensor_msgs::msg::Imu message(const ImuSample &sample, const rclcpp::Time &receipt) {
    sensor_msgs::msg::Imu output;
    output.header.stamp = receipt;
    output.header.frame_id = "imu_link";
    output.angular_velocity.x = sample.angular_velocity[0];
    output.angular_velocity.y = sample.angular_velocity[1];
    output.angular_velocity.z = sample.angular_velocity[2];
    output.angular_velocity_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};
    output.linear_acceleration.x = sample.linear_acceleration[0];
    output.linear_acceleration.y = sample.linear_acceleration[1];
    output.linear_acceleration.z = sample.linear_acceleration[2];
    output.linear_acceleration_covariance = {0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1};
    output.orientation_covariance[0] = -1.0;
    return output;
}
}
