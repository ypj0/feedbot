#include "feedbot_imu_ros2/callback_gap_histogram.hpp"
#include "feedbot_imu_ros2/message.hpp"
#include "feedbot_imu_ros2/latest_imu_publisher.hpp"
#include "serial_port.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <exception>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <thread>

namespace feedbot_imu_ros2 {
class ImuNode : public rclcpp::Node {
    std::unique_ptr<feedbot_chassis::SerialPort> port_;
    GyroStream stream_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
    std::unique_ptr<LatestImuPublisher> latest_publisher_;
    std::unique_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>> realtime_diagnostics_;
    std::thread receiver_thread_;
    std::atomic<bool> stop_receiver_{false};
    std::exception_ptr receiver_error_;
    double last_gyro_ = 0.0;
    double report_started_ = 0.0;
    uint64_t received_ = 0;
    uint64_t handoff_ = 0;
    uint64_t diagnostic_handoff_drop_count_ = 0;
    uint64_t report_received_count_ = 0;
    uint64_t report_handoff_count_ = 0;
    double max_gyro_receipt_gap_ = 0.0;
    double previous_callback_entered_ = 0.0;
    double callback_gap_sum_ = 0.0;
    double callback_gap_max_ = 0.0;
    double read_sum_ = 0.0;
    double read_max_ = 0.0;
    double parse_sum_ = 0.0;
    double parse_max_ = 0.0;
    double publish_sum_ = 0.0;
    double publish_max_ = 0.0;
    double gyro_batch_gap_sum_ = 0.0;
    double gyro_batch_gap_max_ = 0.0;
    double previous_gyro_batch_returned_ = 0.0;
    double last_timeout_checked_ = 0.0;
    uint64_t callback_count_ = 0;
    uint64_t read_count_ = 0;
    uint64_t parse_count_ = 0;
    uint64_t gyro_batch_gap_count_ = 0;
    uint64_t report_callback_count_ = 0;
    uint64_t report_gyro_batch_gap_count_ = 0;
    CallbackGapHistogram callback_gap_histogram_;

    struct AttemptObservation {
        const char *stage = "callback_entry";
        double callback_entered = 0.0;
        double callback_gap = 0.0;
        double queue_started = 0.0;
        double queue_returned = 0.0;
        double read_started = 0.0;
        double read_returned = 0.0;
        double failure_observed = 0.0;
        int queued_bytes = -1;
        int queue_result = -1;
        int queue_errno = 0;
        bool read_attempted = false;
        ssize_t read_bytes = 0;
        int read_errno = 0;
    };
    AttemptObservation attempt_;

    void log_failed_attempt() {
        RCLCPP_ERROR(get_logger(),
                     "IMU failed attempt stage=%s callback_enter=%.9f callback_gap=%.9f "
                     "queue_start=%.9f queue_return=%.9f queue_result=%d queue_errno=%d queued=%d "
                     "read_attempted=%d read_start=%.9f read_return=%.9f read=%zd read_errno=%d "
                     "last_gyro=%.9f timeout_check=%.9f failure_observed=%.9f",
                     attempt_.stage, attempt_.callback_entered, attempt_.callback_gap,
                     attempt_.queue_started, attempt_.queue_returned, attempt_.queue_result,
                     attempt_.queue_errno, attempt_.queued_bytes, attempt_.read_attempted ? 1 : 0,
                     attempt_.read_started, attempt_.read_returned, attempt_.read_bytes,
                     attempt_.read_errno, last_gyro_, last_timeout_checked_, attempt_.failure_observed);
    }

    struct BatchObservation {
        double callback_entered;
        double callback_gap;
        double read_started;
        double read_returned;
        double parse_duration;
        double publish_duration;
        double last_gyro_at_check;
        double timeout_checked;
        int queued_bytes;
        ssize_t read_bytes;
        uint32_t gyro_count;
        uint64_t accepted_packets;
        uint64_t rejected_bytes;
    };
    std::array<BatchObservation, 128> recent_batches_{};
    size_t recent_batch_next_ = 0;
    size_t recent_batch_count_ = 0;

    void remember_batch(const BatchObservation &batch) noexcept {
        recent_batches_[recent_batch_next_] = batch;
        recent_batch_next_ = (recent_batch_next_ + 1) % recent_batches_.size();
        recent_batch_count_ = std::min(recent_batch_count_ + 1, recent_batches_.size());
    }

    void log_recent_batches() {
        RCLCPP_ERROR(get_logger(), "IMU completed callback journal: %zu recent callback batches", recent_batch_count_);
        const auto first = (recent_batch_next_ + recent_batches_.size() - recent_batch_count_) % recent_batches_.size();
        for (size_t offset = 0; offset < recent_batch_count_; ++offset) {
            const auto &batch = recent_batches_[(first + offset) % recent_batches_.size()];
            RCLCPP_ERROR(get_logger(),
                         "IMU batch callback_enter=%.9f callback_gap=%.9f read_start=%.9f "
                         "read_return=%.9f queued=%d read=%zd gyros=%u parse_excluding_publish=%.9f "
                         "publish=%.9f accepted=%llu rejected=%llu last_gyro_at_check=%.9f "
                         "check=%.9f age=%.9f",
                         batch.callback_entered, batch.callback_gap, batch.read_started,
                         batch.read_returned, batch.queued_bytes, batch.read_bytes, batch.gyro_count,
                         batch.parse_duration, batch.publish_duration,
                         static_cast<unsigned long long>(batch.accepted_packets),
                         static_cast<unsigned long long>(batch.rejected_bytes), batch.last_gyro_at_check,
                         batch.timeout_checked, batch.timeout_checked - batch.last_gyro_at_check);
        }
    }

    void report(const std::string &error = "") {
        const auto current = feedbot_chassis::steady_seconds();
        const auto publication = latest_publisher_->stats();
        diagnostic_msgs::msg::DiagnosticArray output;
        output.header.stamp = now();
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "feedbot_imu/WIT";
        status.hardware_id = "/dev/rikiimu";
        status.level = error.empty() ?
            ((publication.overwritten == 0 && diagnostic_handoff_drop_count_ == 0) ? status.OK : status.WARN) :
            status.ERROR;
        status.message = error.empty() ?
            ((publication.overwritten == 0 && diagnostic_handoff_drop_count_ == 0) ?
             "gyro receipt time; absolute alignment and covariance uncalibrated" :
             "IMU pending samples overwritten or diagnostic handoff rejected") : error;
        auto add = [&status](const char *key, auto value) {
            diagnostic_msgs::msg::KeyValue entry;
            entry.key = key;
            entry.value = std::to_string(value);
            status.values.push_back(entry);
        };
        auto add_text = [&status](const char *key, const std::string &value) {
            diagnostic_msgs::msg::KeyValue entry;
            entry.key = key;
            entry.value = value;
            status.values.push_back(entry);
        };
        add("received_gyro_count", received_);
        add("publish_handoff_count", handoff_);
        add("publish_pending_overwrite_count", publication.overwritten);
        add("publish_completed_count", publication.completed);
        add("publish_pending", publication.pending);
        add("publish_in_flight", publication.publishing);
        add("publish_discarded_on_stop_count", publication.discarded_on_stop);
        add("publish_failure_count", publication.failed);
        add_text("publish_policy", "latest_pending");
        add("diagnostic_handoff_drop_count", diagnostic_handoff_drop_count_);
        add("accepted_packet_count", stream_.accepted());
        add("decoder_rejected_bytes", stream_.rejected());
        add("max_gyro_receipt_gap_s", max_gyro_receipt_gap_);
        add("received_gyro_rate_hz", (received_ - report_received_count_) / (current - report_started_));
        add("publish_handoff_rate_hz", (handoff_ - report_handoff_count_) / (current - report_started_));
        add("gyro_age_s", current - last_gyro_);
        const auto callbacks = callback_count_ - report_callback_count_;
        const auto batch_gaps = gyro_batch_gap_count_ - report_gyro_batch_gap_count_;
        add("callback_gap_mean_s", callbacks == 0 ? 0.0 : callback_gap_sum_ / callbacks);
        add("callback_gap_max_s", callback_gap_max_);
        add("callback_gap_histogram_sample_count", callback_gap_histogram_.sample_count());
        add_text("callback_gap_histogram_counts", callback_gap_histogram_.counts_csv());
        add("read_mean_s", read_count_ == 0 ? 0.0 : read_sum_ / read_count_);
        add("read_max_s", read_max_);
        add("parse_excluding_publish_mean_s", parse_count_ == 0 ? 0.0 : parse_sum_ / parse_count_);
        add("parse_excluding_publish_max_s", parse_max_);
        add("publish_handoff_mean_s", received_ == report_received_count_ ? 0.0 : publish_sum_ / (received_ - report_received_count_));
        add("publish_handoff_max_s", publish_max_);
        add("gyro_read_return_batch_gap_mean_s", batch_gaps == 0 ? 0.0 : gyro_batch_gap_sum_ / batch_gaps);
        add("gyro_read_return_batch_gap_max_s", gyro_batch_gap_max_);
        add("timeout_check_steady_s", last_timeout_checked_);
        add("last_gyro_timeout_origin_steady_s", last_gyro_);
        if (!error.empty()) {
            add_text("failure_stage", attempt_.stage);
            add("failure_callback_enter_steady_s", attempt_.callback_entered);
            add("failure_callback_gap_s", attempt_.callback_gap);
            add("failure_queue_query_start_steady_s", attempt_.queue_started);
            add("failure_queue_query_return_steady_s", attempt_.queue_returned);
            add("failure_queue_query_result", attempt_.queue_result);
            add("failure_queue_query_errno", attempt_.queue_errno);
            add("failure_queued_bytes", attempt_.queued_bytes);
            add("failure_read_attempted", attempt_.read_attempted);
            add("failure_read_start_steady_s", attempt_.read_started);
            add("failure_read_return_steady_s", attempt_.read_returned);
            add("failure_read_bytes", attempt_.read_bytes);
            add("failure_read_errno", attempt_.read_errno);
            add("failure_observed_steady_s", attempt_.failure_observed);
        }
        output.status.push_back(status);
        if (error.empty()) {
            if (!realtime_diagnostics_->try_publish(output)) ++diagnostic_handoff_drop_count_;
        } else {
            diagnostics_->publish(output);
        }
        report_started_ = current;
        report_received_count_ = received_;
        report_handoff_count_ = handoff_;
        report_callback_count_ = callback_count_;
        report_gyro_batch_gap_count_ = gyro_batch_gap_count_;
        callback_gap_sum_ = callback_gap_max_ = 0.0;
        callback_gap_histogram_.reset();
        read_sum_ = read_max_ = 0.0;
        read_count_ = 0;
        parse_sum_ = parse_max_ = 0.0;
        parse_count_ = 0;
        publish_sum_ = publish_max_ = 0.0;
        gyro_batch_gap_sum_ = gyro_batch_gap_max_ = 0.0;
    }

    void receive() {
        const auto callback_entered = feedbot_chassis::steady_seconds();
        const auto callback_gap = previous_callback_entered_ == 0.0 ? 0.0 : callback_entered - previous_callback_entered_;
        attempt_ = {};
        attempt_.callback_entered = callback_entered;
        attempt_.callback_gap = callback_gap;
        previous_callback_entered_ = callback_entered;
        ++callback_count_;
        callback_gap_sum_ += callback_gap;
        callback_gap_max_ = std::max(callback_gap_max_, callback_gap);
        if (callback_gap > 0.0) callback_gap_histogram_.observe(callback_gap);
        try {
            attempt_.stage = "publisher_check";
            latest_publisher_->rethrow_error();
            int available = 0;
            attempt_.stage = "queue_query";
            attempt_.queue_started = feedbot_chassis::steady_seconds();
            attempt_.queue_result = ioctl(port_->fd(), FIONREAD, &available);
            attempt_.queue_errno = attempt_.queue_result < 0 ? errno : 0;
            attempt_.queue_returned = feedbot_chassis::steady_seconds();
            if (attempt_.queue_result == 0) attempt_.queued_bytes = available;
            feedbot_chassis::require_serial(attempt_.queue_result == 0, "IMU input queue query failed");
            attempt_.stage = "queue_limit";
            feedbot_chassis::require_serial(available <= 1024, "IMU serial backlog exceeds 1024 bytes");
            uint8_t bytes[1024];
            attempt_.stage = "serial_read";
            const auto read_started = feedbot_chassis::steady_seconds();
            attempt_.read_started = read_started;
            attempt_.read_attempted = true;
            const auto count = ::read(port_->fd(), bytes, sizeof(bytes));
            const auto read_errno = errno;
            const auto read_returned = feedbot_chassis::steady_seconds();
            attempt_.read_returned = read_returned;
            attempt_.read_bytes = count;
            attempt_.read_errno = count < 0 ? read_errno : 0;
            const auto read_duration = read_returned - read_started;
            read_sum_ += read_duration;
            read_max_ = std::max(read_max_, read_duration);
            ++read_count_;
            if (count < 0 && read_errno != EAGAIN && read_errno != EWOULDBLOCK && read_errno != EINTR)
                throw std::runtime_error("IMU serial read failed");
            feedbot_chassis::require_serial(count != 0, "IMU serial disconnected");
            attempt_.stage = "parse_and_handoff";
            double batch_publish_duration = 0.0;
            uint32_t batch_gyro_count = 0;
            for (ssize_t index = 0; index < count; ++index) {
                const auto receipt = feedbot_chassis::steady_seconds();
                const auto sample = stream_.push(bytes[index], receipt);
                if (sample) {
                    if (received_ > 0) max_gyro_receipt_gap_ = std::max(max_gyro_receipt_gap_, receipt - last_gyro_);
                    const auto publish_started = feedbot_chassis::steady_seconds();
                    latest_publisher_->submit(message(*sample, now()));
                    ++handoff_;
                    const auto publish_finished = feedbot_chassis::steady_seconds();
                    const auto publish_duration = publish_finished - publish_started;
                    batch_publish_duration += publish_duration;
                    publish_sum_ += publish_duration;
                    publish_max_ = std::max(publish_max_, publish_duration);
                    last_gyro_ = receipt;
                    ++received_;
                    ++batch_gyro_count;
                }
            }
            const auto current = feedbot_chassis::steady_seconds();
            last_timeout_checked_ = current;
            const auto parse_duration = current - read_returned - batch_publish_duration;
            parse_sum_ += parse_duration;
            parse_max_ = std::max(parse_max_, parse_duration);
            ++parse_count_;
            if (batch_gyro_count > 0) {
                if (previous_gyro_batch_returned_ != 0.0) {
                    const auto batch_gap = read_returned - previous_gyro_batch_returned_;
                    gyro_batch_gap_sum_ += batch_gap;
                    gyro_batch_gap_max_ = std::max(gyro_batch_gap_max_, batch_gap);
                    ++gyro_batch_gap_count_;
                }
                previous_gyro_batch_returned_ = read_returned;
            }
            remember_batch({callback_entered, callback_gap, read_started, read_returned,
                            parse_duration, batch_publish_duration, last_gyro_, current,
                            available, count, batch_gyro_count, stream_.accepted(), stream_.rejected()});
            attempt_.stage = "timeout_check";
            feedbot_chassis::require_serial(current - last_gyro_ < 0.2, "IMU gyro timeout (200 ms)");
            attempt_.stage = "diagnostic_report";
            if (current - report_started_ >= 1.0) report();
        } catch (const std::exception &error) {
            attempt_.failure_observed = feedbot_chassis::steady_seconds();
            port_.reset();
            report(error.what());
            log_failed_attempt();
            log_recent_batches();
            throw;
        }
    }

    void receive_loop() {
        try {
            while (rclcpp::ok() && !stop_receiver_.load(std::memory_order_acquire)) {
                receive();
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        } catch (...) {
            receiver_error_ = std::current_exception();
            if (rclcpp::ok()) rclcpp::shutdown();
        }
    }

public:
    ImuNode() : Node("feedbot_imu") {
        publisher_ = create_publisher<sensor_msgs::msg::Imu>("/imu", rclcpp::SensorDataQoS());
        diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
        latest_publisher_ = std::make_unique<LatestImuPublisher>(
            [this](const auto &output) { publisher_->publish(output); });
        realtime_diagnostics_ = std::make_unique<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>(diagnostics_);
        port_ = std::make_unique<feedbot_chassis::SerialPort>("/dev/rikiimu");
        last_gyro_ = report_started_ = last_timeout_checked_ = feedbot_chassis::steady_seconds();
        RCLCPP_INFO(get_logger(), "IMU started: 115200 8N1, read-only, receipt timestamps, no hardware writes");
        receiver_thread_ = std::thread([this] { receive_loop(); });
    }

    ~ImuNode() override { stop_receiver(); }

    void stop_receiver() {
        stop_receiver_.store(true, std::memory_order_release);
        if (receiver_thread_.joinable()) receiver_thread_.join();
    }

    void finish_receiver() {
        stop_receiver();
        latest_publisher_->stop();
        if (receiver_error_) std::rethrow_exception(receiver_error_);
        latest_publisher_->rethrow_error();
    }
};
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    int result = 0;
    try {
        auto node = std::make_shared<feedbot_imu_ros2::ImuNode>();
        rclcpp::spin(node);
        node->finish_receiver();
    } catch (const std::exception &error) {
        RCLCPP_ERROR(rclcpp::get_logger("feedbot_imu"), "%s", error.what());
        result = 1;
    }
    if (rclcpp::ok()) rclcpp::shutdown();
    return result;
}
