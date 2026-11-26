#include <cerrno>
#include <cstring>
#include <memory>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <robot_localization/ros_filter_types.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.arguments({"ekf_filter_node"});
    options.clock_type(RCL_ROS_TIME);
    auto filter = std::make_shared<robot_localization::RosEkf>(options);
    filter->initialize();
    const auto thread_id = static_cast<id_t>(syscall(SYS_gettid));
    if (setpriority(PRIO_PROCESS, thread_id, -10) != 0) {
        const int error = errno;
        RCLCPP_FATAL(rclcpp::get_logger("feedbot_ekf"),
                     "Cannot set EKF executor nice=-10: %s", std::strerror(error));
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::spin(filter->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
