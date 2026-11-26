#include "feedbot_imu_ros2/latest_imu_publisher.hpp"
#include <cassert>
#include <chrono>
#include <future>
#include <stdexcept>
#include <vector>

using feedbot_imu_ros2::LatestImuPublisher;
using namespace std::chrono_literals;

LatestImuPublisher::Message sample(int sequence) {
    LatestImuPublisher::Message message;
    message.header.stamp.sec = sequence;
    message.header.stamp.nanosec = 123;
    message.angular_velocity.z = sequence;
    message.angular_velocity_covariance[8] = 0.04;
    return message;
}

int main() {
    std::promise<void> entered;
    std::promise<void> release;
    auto released = release.get_future();
    std::promise<void> latest_sent;
    std::vector<LatestImuPublisher::Message> delivered;
    LatestImuPublisher publisher([&](const auto &message) {
        if (message.header.stamp.sec == 1) {
            entered.set_value();
            released.wait();
        }
        delivered.push_back(message);
        if (message.header.stamp.sec == 10) latest_sent.set_value();
    });
    publisher.submit(sample(1));
    assert(entered.get_future().wait_for(1s) == std::future_status::ready);
    auto producer = std::async(std::launch::async, [&] {
        for (int sequence = 2; sequence <= 10; ++sequence) publisher.submit(sample(sequence));
    });
    assert(producer.wait_for(1s) == std::future_status::ready);
    producer.get();
    const auto busy = publisher.stats();
    assert(busy.accepted == 10 && busy.overwritten == 8 && busy.completed == 0);
    assert(busy.pending && busy.publishing);
    release.set_value();
    assert(latest_sent.get_future().wait_for(1s) == std::future_status::ready);
    publisher.stop();
    assert(delivered.size() == 2);
    assert(delivered[0].header.stamp.sec == 1 && delivered[1].header.stamp.sec == 10);
    assert(delivered[1].header.stamp.nanosec == 123);
    assert(delivered[1].angular_velocity.z == 10);
    assert(delivered[1].angular_velocity_covariance[8] == 0.04);
    const auto finished = publisher.stats();
    assert(finished.completed == 2 && !finished.pending && !finished.publishing);
    assert(finished.accepted == finished.completed + finished.overwritten);

    std::promise<void> failure_entered;
    LatestImuPublisher failing([&](const auto &) {
        failure_entered.set_value();
        throw std::runtime_error("injected publish failure");
    });
    failing.submit(sample(1));
    assert(failure_entered.get_future().wait_for(1s) == std::future_status::ready);
    failing.stop();
    assert(failing.stats().failed == 1 && failing.stats().completed == 0);
    bool observed = false;
    try {
        failing.rethrow_error();
    } catch (const std::runtime_error &error) {
        observed = std::string(error.what()) == "injected publish failure";
    }
    assert(observed);

    std::promise<void> stop_entered;
    std::promise<void> stop_release;
    auto stop_released = stop_release.get_future();
    LatestImuPublisher stopping([&](const auto &) {
        stop_entered.set_value();
        stop_released.wait();
    });
    stopping.submit(sample(1));
    assert(stop_entered.get_future().wait_for(1s) == std::future_status::ready);
    stopping.submit(sample(2));
    auto stopper = std::async(std::launch::async, [&] { stopping.stop(); });
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (stopping.stats().discarded_on_stop == 0 && std::chrono::steady_clock::now() < deadline)
        std::this_thread::yield();
    assert(stopping.stats().discarded_on_stop == 1);
    stop_release.set_value();
    assert(stopper.wait_for(1s) == std::future_status::ready);
    stopper.get();
    assert(stopping.stats().completed == 1 && !stopping.stats().pending);
}
