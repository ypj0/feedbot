#include "feedbot_imu_ros2/message.hpp"
#include <cassert>
#include <iostream>

std::array<uint8_t, 11> packet(uint8_t kind, int yaw) {
    std::array<uint8_t, 11> bytes{};
    bytes[0] = 0x55;
    bytes[1] = kind;
    bytes[6] = yaw & 255;
    bytes[7] = (static_cast<uint16_t>(yaw) >> 8) & 255;
    for (size_t index = 0; index < 10; ++index) bytes[10] += bytes[index];
    return bytes;
}

int main() {
    feedbot_imu_ros2::GyroStream stream;
    double receipt = 1.0;
    unsigned updates = 0;
    auto consume = [&](const auto &bytes) {
        for (const auto byte : bytes) {
            const auto gyro = stream.push(byte, receipt += 0.00001);
            if (gyro) {
                ++updates;
                const auto output = feedbot_imu_ros2::message(*gyro, rclcpp::Time(1234567890LL));
                assert(output.header.stamp.sec == 1 && output.header.stamp.nanosec == 234567890);
                assert(output.header.frame_id == "imu_link");
                assert(std::abs(output.angular_velocity.z + 1000.0 * std::acos(-1.0) / 180.0) < 1e-10);
                assert(output.angular_velocity_covariance[8] == 0.01);
                assert(output.orientation_covariance[0] == -1 && output.linear_acceleration_covariance[0] == -1);
            }
        }
    };
    consume(packet(0x51, 0));
    consume(packet(0x53, 0));
    assert(updates == 0);
    consume(packet(0x52, -16384));
    assert(updates == 1);
    consume(packet(0x51, 0));
    consume(packet(0x53, 0));
    assert(updates == 1);
    auto damaged = packet(0x52, 16384);
    damaged[10] ^= 1;
    consume(damaged);
    assert(updates == 1 && stream.rejected() > 0);
    consume(packet(0x52, -16384));
    assert(updates == 2);
    std::cout << "PASS fresh gyro only, no channel replay, units, sign, receipt stamp, covariance, checksum rejection\n";
}
