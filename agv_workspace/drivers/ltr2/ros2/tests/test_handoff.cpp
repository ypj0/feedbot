#include "ltr2_ros2/message.hpp"
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <thread>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("scan_handoff_test");
    auto publisher = node->create_publisher<sensor_msgs::msg::LaserScan>(
        "/scan_handoff_test", rclcpp::QoS(1).transient_local());
    using Output = realtime_tools::RealtimePublisher<sensor_msgs::msg::LaserScan>;
    auto output = std::make_unique<Output>(publisher);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!output->can_publish() && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    assert(output->can_publish());
    std::mutex mutex;
    std::condition_variable condition;
    bool held = false;
    bool release = false;
    std::thread busy_worker([&] {
        std::unique_lock<std::mutex> output_lock(output->get_mutex());
        std::unique_lock<std::mutex> lock(mutex);
        held = true;
        condition.notify_one();
        condition.wait(lock, [&] { return release; });
    });
    {
        std::unique_lock<std::mutex> lock(mutex);
        condition.wait(lock, [&] { return held; });
    }
    ltr2::Scan scan{};
    scan.ranges.assign(1075, 1.0F);
    scan.intensities.assign(1075, 100.0F);
    const auto message = ltr2_ros2::to_message(scan, rclcpp::Time(123, 456, RCL_SYSTEM_TIME));
    const auto started = std::chrono::steady_clock::now();
    assert(!output->try_publish(message));
    assert(std::chrono::steady_clock::now() - started < std::chrono::milliseconds(100));
    {
        std::lock_guard<std::mutex> lock(mutex);
        release = true;
    }
    condition.notify_one();
    busy_worker.join();
    assert(output->get_msg().ranges.empty());
    unsigned received = 0;
    auto subscription = node->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan_handoff_test", rclcpp::QoS(1).transient_local(),
        [&](sensor_msgs::msg::LaserScan::ConstSharedPtr sample) {
            assert(sample->header.stamp.sec == 123 && sample->header.stamp.nanosec == 456);
            assert(sample->ranges == message.ranges && sample->intensities == message.intensities);
            std::lock_guard<std::mutex> lock(mutex);
            ++received;
            condition.notify_one();
        });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&] { executor.spin(); });
    assert(output->try_publish(message));
    {
        std::unique_lock<std::mutex> lock(mutex);
        assert(condition.wait_for(lock, std::chrono::seconds(3), [&] { return received == 1; }));
    }
    output.reset();
    {
        std::unique_lock<std::mutex> lock(mutex);
        assert(!condition.wait_for(lock, std::chrono::milliseconds(350), [&] { return received > 1; }));
    }
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
}
