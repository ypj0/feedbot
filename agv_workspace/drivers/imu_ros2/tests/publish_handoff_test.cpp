#include <cassert>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <thread>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("publish_handoff_test");
    const auto qos = rclcpp::QoS(1).transient_local();
    auto publisher = node->create_publisher<sensor_msgs::msg::Imu>("/publish_handoff_test", qos);
    realtime_tools::RealtimePublisher<sensor_msgs::msg::Imu> realtime_publisher(publisher);

    const auto startup_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!realtime_publisher.can_publish() && std::chrono::steady_clock::now() < startup_deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    assert(realtime_publisher.can_publish());

    {
        std::lock_guard<std::mutex> lock(realtime_publisher.get_mutex());
        realtime_publisher.msg_.header.stamp.sec = 42;
    }
    std::mutex coordination_mutex;
    std::condition_variable coordination;
    bool locked = false;
    bool release = false;
    std::thread holder([&] {
        std::unique_lock<std::mutex> publish_lock(realtime_publisher.get_mutex());
        std::unique_lock<std::mutex> coordination_lock(coordination_mutex);
        locked = true;
        coordination.notify_one();
        coordination.wait(coordination_lock, [&] { return release; });
    });
    {
        std::unique_lock<std::mutex> lock(coordination_mutex);
        coordination.wait(lock, [&] { return locked; });
    }

    sensor_msgs::msg::Imu rejected;
    rejected.header.stamp.sec = 99;
    const auto started = std::chrono::steady_clock::now();
    assert(!realtime_publisher.try_publish(rejected));
    const auto elapsed = std::chrono::steady_clock::now() - started;
    assert(elapsed < std::chrono::milliseconds(100));
    {
        std::lock_guard<std::mutex> lock(coordination_mutex);
        release = true;
    }
    coordination.notify_one();
    holder.join();
    assert(realtime_publisher.get_msg().header.stamp.sec == 42);

    std::mutex received_mutex;
    std::condition_variable received_condition;
    int32_t received_stamp = 0;
    auto subscription = node->create_subscription<sensor_msgs::msg::Imu>(
        "/publish_handoff_test", qos,
        [&](sensor_msgs::msg::Imu::ConstSharedPtr message) {
            std::lock_guard<std::mutex> lock(received_mutex);
            received_stamp = message->header.stamp.sec;
            received_condition.notify_one();
        });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&] { executor.spin(); });

    sensor_msgs::msg::Imu accepted;
    accepted.header.stamp.sec = 123;
    assert(realtime_publisher.try_publish(accepted));
    {
        std::unique_lock<std::mutex> lock(received_mutex);
        assert(received_condition.wait_for(
            lock, std::chrono::seconds(3), [&] { return received_stamp == 123; }));
    }

    executor.cancel();
    spinner.join();
    executor.remove_node(node);
    subscription.reset();
    rclcpp::shutdown();
}
