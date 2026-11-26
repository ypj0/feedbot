#pragma once

#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
#include <mutex>
#include <optional>
#include <sensor_msgs/msg/imu.hpp>
#include <stdexcept>
#include <thread>
#include <utility>

namespace feedbot_imu_ros2 {
class LatestImuPublisher {
public:
    using Message = sensor_msgs::msg::Imu;
    struct Stats {
        uint64_t accepted = 0;
        uint64_t overwritten = 0;
        uint64_t completed = 0;
        uint64_t discarded_on_stop = 0;
        uint64_t failed = 0;
        bool pending = false;
        bool publishing = false;
    };

    explicit LatestImuPublisher(std::function<void(const Message &)> publish)
        : publish_(std::move(publish)), worker_([this] { run(); }) {}

    ~LatestImuPublisher() { stop(); }

    void submit(Message message) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (error_) std::rethrow_exception(error_);
            if (stopping_) throw std::runtime_error("IMU publisher stopped");
            if (pending_) ++stats_.overwritten;
            pending_ = std::move(message);
            ++stats_.accepted;
        }
        ready_.notify_one();
    }

    Stats stats() const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto result = stats_;
        result.pending = pending_.has_value();
        return result;
    }

    void rethrow_error() const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (error_) std::rethrow_exception(error_);
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
            if (pending_) {
                ++stats_.discarded_on_stop;
                pending_.reset();
            }
        }
        ready_.notify_one();
        if (worker_.joinable()) worker_.join();
    }

private:
    void run() {
        std::unique_lock<std::mutex> lock(mutex_);
        while (true) {
            ready_.wait(lock, [this] { return stopping_ || pending_.has_value(); });
            if (stopping_) return;
            auto message = std::move(*pending_);
            pending_.reset();
            stats_.publishing = true;
            lock.unlock();
            try {
                publish_(message);
            } catch (...) {
                lock.lock();
                error_ = std::current_exception();
                ++stats_.failed;
                stats_.publishing = false;
                stopping_ = true;
                if (pending_) {
                    ++stats_.discarded_on_stop;
                    pending_.reset();
                }
                return;
            }
            lock.lock();
            ++stats_.completed;
            stats_.publishing = false;
        }
    }

    std::function<void(const Message &)> publish_;
    mutable std::mutex mutex_;
    std::condition_variable ready_;
    std::optional<Message> pending_;
    Stats stats_;
    std::exception_ptr error_;
    bool stopping_ = false;
    std::thread worker_;
};
}
