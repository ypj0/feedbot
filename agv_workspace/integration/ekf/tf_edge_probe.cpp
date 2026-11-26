#include <iomanip>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <tf2_msgs/msg/tf_message.hpp>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("feedbot_tf_edge_probe");
    auto record = [](const std::string &topic, const tf2_msgs::msg::TFMessage &message,
                     const rclcpp::MessageInfo &info) {
        std::ostringstream gid;
        gid << std::hex << std::setfill('0');
        for (const auto byte : info.get_rmw_message_info().publisher_gid.data)
            gid << std::setw(2) << unsigned(byte);
        for (const auto &transform : message.transforms) {
            const auto stamp = int64_t(transform.header.stamp.sec) * 1000000000 + transform.header.stamp.nanosec;
            std::cout << "{\"topic\":" << std::quoted(topic)
                      << ",\"stamp_ns\":" << stamp
                      << ",\"parent\":" << std::quoted(transform.header.frame_id)
                      << ",\"child\":" << std::quoted(transform.child_frame_id)
                      << ",\"gid\":" << std::quoted(gid.str()) << "}" << std::endl;
        }
    };
    auto dynamic = node->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 100,
        [&record](const tf2_msgs::msg::TFMessage::SharedPtr message, const rclcpp::MessageInfo &info) {
            record("/tf", *message, info);
        });
    auto fixed = node->create_subscription<tf2_msgs::msg::TFMessage>("/tf_static", rclcpp::QoS(100).transient_local(),
        [&record](const tf2_msgs::msg::TFMessage::SharedPtr message, const rclcpp::MessageInfo &info) {
            record("/tf_static", *message, info);
        });
    rclcpp::spin(node);
    if (rclcpp::ok()) rclcpp::shutdown();
}
