#include "ltr2_ros2/message.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

static void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

int main() {
    try {
        ltr2::Assembler assembler;
        ltr2::Scan scan{};
        for (unsigned index = 0; index < 8; ++index) {
            ltr2::Block block{};
            block.frame = 10;
            block.index = index;
            block.stamp = 100000 + index * 8333;
            for (unsigned beam = 0; beam < 256; ++beam) {
                block.ranges[beam] = 1000 + index * 256 + beam;
                block.intensities[beam] = index * 256 + beam;
            }
            const bool complete = assembler.add(block, 1000 + index * 0.008333,
                                                 100 + index * 0.008333, scan);
            require(complete == (index == 7), "partial frame emitted");
        }
        const rclcpp::Time received(1234, 567890123, RCL_SYSTEM_TIME);
        const auto message = ltr2_ros2::to_message(scan, received);
        require(rclcpp::Time(message.header.stamp).nanoseconds() == received.nanoseconds(), "receipt stamp changed");
        require(std::abs(received.seconds() - scan.stamp) > 1, "test did not distinguish core stamp");
        require(message.header.frame_id == "scan", "frame id changed");
        require(message.ranges == scan.ranges && message.intensities == scan.intensities, "beam arrays changed");
        require(message.ranges.size() == 1075, "crop count changed");
        require(std::abs(double(message.angle_max) - message.angle_min -
                         (message.ranges.size() - 1) * double(message.angle_increment)) < 1e-6,
                "geometry mismatch");
        require(message.scan_time == float(scan.scan_time) &&
                message.time_increment == float(scan.time_increment), "timing fields changed");
        require(message.range_min == 0.05F && message.range_max == 40.0F, "range limits changed");
        std::cout << "ROS2 conversion, receipt stamp, 1075 beam alignment and geometry passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
