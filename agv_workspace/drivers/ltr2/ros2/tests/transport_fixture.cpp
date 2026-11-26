#include "ltr2_ros2/message.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ltr2_transport_fixture");
    auto publisher = node->create_publisher<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS());
    publisher->on_activate();
    ltr2::Scan scan{};
    scan.angle_increment = 2 * std::acos(-1) / 2048;
    scan.angle_min = -537 * scan.angle_increment;
    scan.angle_max = 537 * scan.angle_increment;
    scan.ranges.assign(1075, 1.0F);
    scan.intensities.assign(1075, 100.0F);
    scan.scan_time = 1.0 / 15;
    scan.time_increment = scan.scan_time / 2048;
    rclcpp::Clock clock(RCL_SYSTEM_TIME);
    rclcpp::WallRate rate(15);
    for (unsigned frame = 0; frame < 180 && rclcpp::ok(); ++frame) {
        publisher->publish(ltr2_ros2::to_message(scan, clock.now()));
        rate.sleep();
    }
    rclcpp::shutdown();
}
