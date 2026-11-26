#pragma once
#include "odometry.hpp"
#include "serial_port.hpp"
#include <algorithm>
#include <iomanip>
#include <poll.h>
#include <sstream>
#include <vector>

namespace feedbot_chassis_ros2 {
using feedbot_chassis::Frame;
using feedbot_chassis::Feedback;

struct Wheels {
    static constexpr double radius = 0.085;
    static constexpr double separation = 0.455;
    double left = 0.0;
    double right = 0.0;

    void accumulate(const Feedback &feedback) {
        const auto sample = feedbot_chassis::wheel_sample(feedback, {});
        left += sample.left_m / radius;
        right += sample.right_m / radius;
    }

    static Frame command(double left_radps, double right_radps) {
        const double linear = radius * (left_radps + right_radps) / 2.0;
        const double angular = radius * (right_radps - left_radps) / separation;
        return feedbot_chassis::velocity_frame({}, linear, angular);
    }
};

struct TxAttempt {
    uint64_t sequence;
    uint64_t frame_sequence;
    Frame frame;
    size_t offset;
    ssize_t returned;
    int error;
    double started;
    double finished;
    bool stopping;
};

class Session {
    int fd_;
    feedbot_chassis::FrameStream parser_;
    uint64_t observed_rejected_ = 0;
    double last_feedback_ = feedbot_chassis::steady_seconds();
    bool output_failed_ = false;
    bool stopped_ = false;
    std::array<TxAttempt, 128> tx_history_{};
    uint64_t tx_attempt_count_ = 0;
    uint64_t tx_frame_count_ = 0;

    void send(const Frame &frame) {
        const auto frame_sequence = ++tx_frame_count_;
        const double deadline = feedbot_chassis::steady_seconds() + 0.04;
        size_t offset = 0;
        try {
            while (offset < frame.size()) {
                const double remaining = deadline - feedbot_chassis::steady_seconds();
                feedbot_chassis::require_serial(remaining > 0, "serial write deadline (40 ms)");
                const double write_started = feedbot_chassis::steady_seconds();
                const auto count = ::write(fd_, frame.data() + offset, frame.size() - offset);
                const int write_error = count < 0 ? errno : 0;
                const double write_finished = feedbot_chassis::steady_seconds();
                tx_history_[tx_attempt_count_ % tx_history_.size()] = {
                    tx_attempt_count_ + 1, frame_sequence, frame, offset, count,
                    write_error, write_started, write_finished, stopped_};
                ++tx_attempt_count_;
                if (count > 0) {
                    offset += static_cast<size_t>(count);
                    continue;
                }
                if (count < 0 && write_error == EINTR) continue;
                feedbot_chassis::require_serial(count < 0 && (write_error == EAGAIN || write_error == EWOULDBLOCK),
                                               "serial write failed");
                pollfd descriptor{fd_, POLLOUT, 0};
                const auto ready = ::poll(&descriptor, 1, static_cast<int>(std::ceil(remaining * 1000)));
                feedbot_chassis::require_serial(ready >= 0 || errno == EINTR, "serial write poll failed");
                feedbot_chassis::require_serial(!(descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)),
                                               "serial output disconnected");
            }
        } catch (...) {
            output_failed_ = true;
            throw;
        }
    }

public:
    Wheels wheels;
    Feedback status{};
    uint64_t feedback_count = 0;
    uint64_t feedback_timeout_count = 0;
    uint64_t runtime_field_error_count = 0;
    uint64_t command_zero_count = 0;
    uint64_t command_nonzero_count = 0;
    double receive_total_last_us = 0.0;
    double receive_total_max_us = 0.0;
    double read_syscall_total_last_us = 0.0;
    double read_syscall_total_max_us = 0.0;
    double parse_validate_total_last_us = 0.0;
    double parse_validate_total_max_us = 0.0;
    Frame last_observed_frame{};
    bool last_observed_frame_valid = false;
    unsigned last_observed_frame_type = 0;
    unsigned last_observed_frame_ack = 0;
    double last_observed_frame_monotonic_s = 0.0;
    double first_feedback_monotonic_s = 0.0;
    double first_read_return_monotonic_s = 0.0;
    double last_rejection_monotonic = 0.0;
    double last_receive_fault_monotonic = 0.0;

    explicit Session(int descriptor) : fd_(descriptor) {}

    uint64_t rejected() const { return parser_.rejected; }

    uint64_t tx_attempt_count() const { return tx_attempt_count_; }

    std::vector<TxAttempt> tx_history() const {
        std::vector<TxAttempt> history;
        const auto count = std::min<uint64_t>(tx_attempt_count_, tx_history_.size());
        history.reserve(count);
        for (uint64_t index = tx_attempt_count_ - count; index < tx_attempt_count_; ++index) {
            history.push_back(tx_history_[index % tx_history_.size()]);
        }
        return history;
    }

    std::string tx_history_json() const {
        std::ostringstream output;
        output << std::setprecision(17) << "[";
        bool first = true;
        for (const auto &attempt : tx_history()) {
            if (!first) output << ",";
            first = false;
            output << "{\"sequence\":" << attempt.sequence
                   << ",\"frame_sequence\":" << attempt.frame_sequence
                   << ",\"offset\":" << attempt.offset
                   << ",\"requested\":" << attempt.frame.size() - attempt.offset
                   << ",\"returned\":" << attempt.returned
                   << ",\"errno\":" << attempt.error
                   << ",\"started_monotonic_s\":" << attempt.started
                   << ",\"finished_monotonic_s\":" << attempt.finished
                   << ",\"purpose\":\"" << (attempt.stopping ? "stop" : "command")
                   << "\",\"frame_hex\":\"";
            output << std::hex << std::setfill('0');
            for (const auto byte : attempt.frame) output << std::setw(2) << unsigned(byte);
            output << std::dec << "\"}";
        }
        output << "]";
        return output.str();
    }

    std::string observed_frame_hex() const {
        if (!last_observed_frame_valid) return "";
        std::ostringstream frame_hex;
        frame_hex << std::hex << std::setfill('0');
        for (const auto byte : last_observed_frame) frame_hex << std::setw(2) << unsigned(byte);
        return frame_hex.str();
    }

    void receive() {
        const double receive_started = feedbot_chassis::steady_seconds();
        first_read_return_monotonic_s = 0.0;
        double read_syscall_total_us = 0.0;
        double parse_validate_total_us = 0.0;
        auto record_timing = [&]() {
            receive_total_last_us = (feedbot_chassis::steady_seconds() - receive_started) * 1e6;
            receive_total_max_us = std::max(receive_total_max_us, receive_total_last_us);
            read_syscall_total_last_us = read_syscall_total_us;
            read_syscall_total_max_us = std::max(read_syscall_total_max_us, read_syscall_total_last_us);
            parse_validate_total_last_us = parse_validate_total_us;
            parse_validate_total_max_us = std::max(parse_validate_total_max_us, parse_validate_total_last_us);
        };
        try {
            feedbot_chassis::require_serial(!stopped_, "serial session stopped");
            uint8_t bytes[1024];
            for (;;) {
                const double read_started = feedbot_chassis::steady_seconds();
                const auto count = ::read(fd_, bytes, sizeof(bytes));
                const int read_errno = errno;
                if (first_read_return_monotonic_s == 0.0) {
                    first_read_return_monotonic_s = feedbot_chassis::steady_seconds();
                }
                read_syscall_total_us += (feedbot_chassis::steady_seconds() - read_started) * 1e6;
                if (count < 0 && read_errno == EINTR) continue;
                if (count < 0 && (read_errno == EAGAIN || read_errno == EWOULDBLOCK)) break;
                feedbot_chassis::require_serial(count > 0, "serial input disconnected");
                const double parse_started = feedbot_chassis::steady_seconds();
                try {
                    for (ssize_t index = 0; index < count; ++index) {
                        parser_.push(bytes[index], [this](const Frame &frame) {
                            last_observed_frame = frame;
                            last_observed_frame_valid = true;
                            last_observed_frame_type = frame[3];
                            last_observed_frame_ack = frame[4];
                            last_observed_frame_monotonic_s = feedbot_chassis::steady_seconds();
                            try {
                                const auto feedback = feedbot_chassis::feedback(frame);
                                wheels.accumulate(feedback);
                                status = feedback;
                                if (feedback_count == 0) {
                                    first_feedback_monotonic_s = feedbot_chassis::steady_seconds();
                                }
                                ++feedback_count;
                                last_feedback_ = feedbot_chassis::steady_seconds();
                            } catch (const std::exception &error) {
                                ++runtime_field_error_count;
                                last_receive_fault_monotonic = feedbot_chassis::steady_seconds();
                                throw std::runtime_error(std::string(error.what()) + "; frame_hex=" +
                                                         observed_frame_hex());
                            }
                        });
                        if (parser_.rejected != observed_rejected_) {
                            observed_rejected_ = parser_.rejected;
                            last_rejection_monotonic = feedbot_chassis::steady_seconds();
                        }
                    }
                } catch (...) {
                    parse_validate_total_us +=
                        (feedbot_chassis::steady_seconds() - parse_started) * 1e6;
                    throw;
                }
                parse_validate_total_us += (feedbot_chassis::steady_seconds() - parse_started) * 1e6;
            }
            if (feedbot_chassis::steady_seconds() - last_feedback_ >= 0.2) {
                ++feedback_timeout_count;
                last_receive_fault_monotonic = feedbot_chassis::steady_seconds();
                throw std::runtime_error("feedback timeout (200 ms)");
            }
        } catch (...) {
            record_timing();
            throw;
        }
        record_timing();
    }

    void command(double left_radps, double right_radps, bool allow_charge_motion = false) {
        feedbot_chassis::require_serial(!stopped_ && !output_failed_, "serial session not writable");
        if (left_radps != 0.0 || right_radps != 0.0) {
            // 真正的安全故障：无新鲜反馈，或急停/报警/关断，必须抛异常并停机。
            feedbot_chassis::require_serial(feedback_count > 0 &&
                feedbot_chassis::steady_seconds() - last_feedback_ < 0.2,
                "nonzero command without fresh feedback");
            feedbot_chassis::require_serial(status.emergency == 0 &&
                status.left_alarm == 0 && status.right_alarm == 0 && status.shutdown == 0,
                "nonzero command blocked by safety status");
            // 充电状态只是“运动抑制”，不是故障：非脱桩模式静默清零，
            // 脱桩/入桩模式允许低速 ±0.20 m/s（原厂出桩约 0.15 m/s），超范围清零。
            // 关键：不因此抛异常，保持串口通信、里程计和诊断持续更新。
            if (status.charge != 0) {
                const double linear = Wheels::radius * (left_radps + right_radps) / 2.0;
                const bool allowed =
                    allow_charge_motion && linear >= -0.20 && linear <= 0.20;
                if (!allowed) {
                    left_radps = 0.0;
                    right_radps = 0.0;
                }
            }
        }
        const auto frame = Wheels::command(left_radps, right_radps);
        send(frame);
        if (left_radps == 0.0 && right_radps == 0.0) ++command_zero_count;
        else ++command_nonzero_count;
    }

    void stop() {
        if (stopped_) return;
        stopped_ = true;
        feedbot_chassis::require_serial(!output_failed_, "output failed; cannot confirm MCU stop");
        send(feedbot_chassis::kReviewedStationary);
    }
};
}
