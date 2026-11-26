#include "feedbot_chassis_ros2/release_connection.hpp"
#include <algorithm>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>

namespace feedbot_chassis_ros2 {
using CallbackReturn = hardware_interface::CallbackReturn;
using Return = hardware_interface::return_type;

class ChassisSystem : public hardware_interface::SystemInterface {
    std::unique_ptr<feedbot_chassis::SerialPort> port_;
    std::unique_ptr<Session> session_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
    std::unique_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>> realtime_diagnostics_;
    uint64_t diagnostic_handoff_drop_count_ = 0;
    double next_diagnostic_ = 0.0;
    bool faulted_ = false;
    bool allow_charge_motion_ = false;
    double joint_update_last_us_ = 0.0;
    double joint_update_max_us_ = 0.0;
    double diagnostic_build_last_us_ = 0.0;
    double diagnostic_build_max_us_ = 0.0;
    double diagnostic_publish_last_us_ = 0.0;
    double diagnostic_publish_max_us_ = 0.0;
    double activation_started_monotonic_s_ = 0.0;
    double read_callback_entry_monotonic_s_ = 0.0;
    double read_callback_finished_monotonic_s_ = 0.0;
    double read_callback_last_us_ = 0.0;
    double read_callback_last_complete_us_ = 0.0;
    double read_callback_previous_complete_us_ = 0.0;
    double read_callback_max_complete_us_ = 0.0;

    void clear_commands() {
        set_command("left_wheel_joint/velocity", 0.0);
        set_command("right_wheel_joint/velocity", 0.0);
    }

    void finish_read_timing(double entry_monotonic_s, bool complete) {
        read_callback_entry_monotonic_s_ = entry_monotonic_s;
        read_callback_finished_monotonic_s_ = feedbot_chassis::steady_seconds();
        read_callback_last_us_ =
            (read_callback_finished_monotonic_s_ - read_callback_entry_monotonic_s_) * 1e6;
        if (complete) {
            read_callback_previous_complete_us_ = read_callback_last_complete_us_;
            read_callback_last_complete_us_ = read_callback_last_us_;
            read_callback_max_complete_us_ =
                std::max(read_callback_max_complete_us_, read_callback_last_complete_us_);
        }
    }

    void report(const std::string &error = "", const Session *snapshot = nullptr) {
        const auto *observed = snapshot ? snapshot : session_.get();
        const double build_started = feedbot_chassis::steady_seconds();
        diagnostic_msgs::msg::DiagnosticArray message;
        message.header.stamp = get_node()->now();
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "feedbot_chassis/MCU";
        status.hardware_id = "/dev/rikibase";
        status.level = error.empty() ? status.OK : status.ERROR;
        status.message = error.empty() ? "feedback received; distance scale pending ground calibration" : error;
        auto add_text = [&status](const char *key, const std::string &value) {
            diagnostic_msgs::msg::KeyValue entry;
            entry.key = key;
            entry.value = value;
            status.values.push_back(entry);
        };
        if (observed) {
            const auto &feedback = observed->status;
            auto add = [&status](const char *key, auto value) {
                diagnostic_msgs::msg::KeyValue entry;
                entry.key = key;
                entry.value = std::to_string(value);
                status.values.push_back(entry);
            };
            add("emergency", feedback.emergency);
            add("charge", feedback.charge);
            add("left_alarm", feedback.left_alarm);
            add("right_alarm", feedback.right_alarm);
            add("motor_free", feedback.motor_free);
            add("initialized", feedback.initialized);
            add("shutdown", feedback.shutdown);
            add("power", feedback.power);
            add("version", feedback.version);
            add("feedback_count", observed->feedback_count);
            add("rejected_bytes", observed->rejected());
            add("sync_checksum_rejected_bytes", observed->rejected());
            add("feedback_timeout_count", observed->feedback_timeout_count);
            add("runtime_field_error_count", observed->runtime_field_error_count);
            add("command_zero_count", observed->command_zero_count);
            add("command_nonzero_count", observed->command_nonzero_count);
            add("tx_attempt_count", observed->tx_attempt_count());
            if (!error.empty()) add_text("tx_history_json", observed->tx_history_json());
            add("receive_total_last_us", observed->receive_total_last_us);
            add("receive_total_max_us", observed->receive_total_max_us);
            add("read_syscall_total_last_us", observed->read_syscall_total_last_us);
            add("read_syscall_total_max_us", observed->read_syscall_total_max_us);
            add("parse_validate_total_last_us", observed->parse_validate_total_last_us);
            add("parse_validate_total_max_us", observed->parse_validate_total_max_us);
            add("last_observed_frame_type", observed->last_observed_frame_type);
            add("last_observed_frame_ack", observed->last_observed_frame_ack);
            add("last_observed_frame_monotonic_s", observed->last_observed_frame_monotonic_s);
            add("first_feedback_monotonic_s", observed->first_feedback_monotonic_s);
            add("first_read_return_monotonic_s", observed->first_read_return_monotonic_s);
            add("last_rejection_monotonic_s", observed->last_rejection_monotonic);
            add("last_receive_fault_monotonic_s", observed->last_receive_fault_monotonic);
            diagnostic_msgs::msg::KeyValue frame_hex;
            frame_hex.key = "last_observed_frame_hex";
            frame_hex.value = observed->observed_frame_hex();
            status.values.push_back(frame_hex);
            if (error.empty() && (feedback.left_alarm || feedback.right_alarm)) status.level = status.ERROR;
        }
        if (error.empty() && status.level == status.OK && diagnostic_handoff_drop_count_ > 0) {
            status.level = status.WARN;
            status.message = "diagnostic realtime handoff rejected";
        }
        auto add_timing = [&status](const char *key, double value) {
            diagnostic_msgs::msg::KeyValue entry;
            entry.key = key;
            entry.value = std::to_string(value);
            status.values.push_back(entry);
        };
        add_timing("joint_update_last_us", joint_update_last_us_);
        add_timing("joint_update_max_us", joint_update_max_us_);
        add_timing("diagnostic_build_last_us", diagnostic_build_last_us_);
        add_timing("diagnostic_build_max_us", diagnostic_build_max_us_);
        add_timing("diagnostic_publish_last_us", diagnostic_publish_last_us_);
        add_timing("diagnostic_publish_max_us", diagnostic_publish_max_us_);
        add_timing("activation_started_monotonic_s", activation_started_monotonic_s_);
        add_timing("read_callback_entry_monotonic_s", read_callback_entry_monotonic_s_);
        add_timing("read_callback_finished_monotonic_s", read_callback_finished_monotonic_s_);
        add_timing("read_callback_last_us", read_callback_last_us_);
        add_timing("read_callback_last_complete_us", read_callback_last_complete_us_);
        add_timing("read_callback_previous_complete_us", read_callback_previous_complete_us_);
        add_timing("read_callback_max_complete_us", read_callback_max_complete_us_);
        add_text("diagnostic_handoff_drop_count", std::to_string(diagnostic_handoff_drop_count_));
        add_text("diagnostic_publish_method", error.empty() ?
                 "realtime_try_handoff_not_dds_completion" : "synchronous_error_publish");
        message.status.push_back(status);
        const double build_us = (feedbot_chassis::steady_seconds() - build_started) * 1e6;
        const double publish_started = feedbot_chassis::steady_seconds();
        double publish_us = 0.0;
        try {
            if (error.empty()) {
                if (!realtime_diagnostics_->try_publish(message)) ++diagnostic_handoff_drop_count_;
            } else {
                diagnostics_->publish(message);
            }
            publish_us = (feedbot_chassis::steady_seconds() - publish_started) * 1e6;
        } catch (...) {
            publish_us = (feedbot_chassis::steady_seconds() - publish_started) * 1e6;
            diagnostic_build_last_us_ = build_us;
            diagnostic_build_max_us_ = std::max(diagnostic_build_max_us_, build_us);
            diagnostic_publish_last_us_ = publish_us;
            diagnostic_publish_max_us_ = std::max(diagnostic_publish_max_us_, publish_us);
            throw;
        }
        diagnostic_build_last_us_ = build_us;
        diagnostic_build_max_us_ = std::max(diagnostic_build_max_us_, build_us);
        diagnostic_publish_last_us_ = publish_us;
        diagnostic_publish_max_us_ = std::max(diagnostic_publish_max_us_, publish_us);
    }

    bool release(const std::string &fault_message = "") {
        auto released = release_connection(session_, port_);
        clear_commands();
        if (fault_message.empty() && !released.stop_error) return true;
        std::string message = fault_message;
        if (released.stop_error) {
            try { std::rethrow_exception(released.stop_error); }
            catch (const std::exception &error) {
                if (!message.empty()) message += "; ";
                message += std::string("Stop unconfirmed: ") + error.what();
            }
        }
        RCLCPP_ERROR(get_logger(), "%s", message.c_str());
        try { report(message, released.snapshot.get()); }
        catch (const std::exception &error) {
            RCLCPP_ERROR(get_logger(), "Fault diagnostics failed after serial release: %s", error.what());
            return false;
        }
        return !released.stop_error;
    }

    Return fault(const std::exception &error) {
        faulted_ = true;
        release(error.what());
        return Return::ERROR;
    }

public:
    CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override {
        if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
        const auto &joints = params.hardware_info.joints;
        if (joints.size() != 2) return CallbackReturn::ERROR;
        for (size_t index = 0; index < joints.size(); ++index) {
            const auto &joint = joints[index];
            const std::string expected = index == 0 ? "left_wheel_joint" : "right_wheel_joint";
            if (joint.name != expected || joint.command_interfaces.size() != 1 ||
                joint.command_interfaces[0].name != "velocity" || joint.state_interfaces.size() != 1 ||
                joint.state_interfaces[0].name != "position") {
                RCLCPP_ERROR(get_logger(), "Expected left/right wheel velocity commands and position feedback");
                return CallbackReturn::ERROR;
            }
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
        if (faulted_) return CallbackReturn::ERROR;
        get_node()->get_parameter_or("allow_charge_motion", allow_charge_motion_, false);
        diagnostics_ = get_node()->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
        realtime_diagnostics_ = std::make_unique<
            realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>>(diagnostics_);
        clear_commands();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
        if (faulted_) return CallbackReturn::ERROR;
        activation_started_monotonic_s_ = feedbot_chassis::steady_seconds();
        try {
            clear_commands();
            port_ = std::make_unique<feedbot_chassis::SerialPort>("/dev/rikibase");
            port_->handshake();
            RCLCPP_INFO(get_logger(), "Handshake accepted: 115200 8N1, type=1, byte4=0x60");
            session_ = std::make_unique<Session>(port_->fd());
            session_->command(0.0, 0.0);
            while (session_->feedback_count == 0) {
                session_->receive();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            const double joint_update_started = feedbot_chassis::steady_seconds();
            try {
                set_state("left_wheel_joint/position", session_->wheels.left);
                set_state("right_wheel_joint/position", session_->wheels.right);
            } catch (...) {
                joint_update_last_us_ =
                    (feedbot_chassis::steady_seconds() - joint_update_started) * 1e6;
                joint_update_max_us_ = std::max(joint_update_max_us_, joint_update_last_us_);
                throw;
            }
            joint_update_last_us_ = (feedbot_chassis::steady_seconds() - joint_update_started) * 1e6;
            joint_update_max_us_ = std::max(joint_update_max_us_, joint_update_last_us_);
            next_diagnostic_ = 0.0;
            report();
            return CallbackReturn::SUCCESS;
        } catch (const std::exception &error) {
            fault(error);
            return CallbackReturn::ERROR;
        }
    }

    Return read(const rclcpp::Time &, const rclcpp::Duration &) override {
        const double read_callback_started = feedbot_chassis::steady_seconds();
        if (!session_) return faulted_ ? Return::ERROR : Return::OK;
        try {
            session_->receive();
            const double joint_update_started = feedbot_chassis::steady_seconds();
            try {
                set_state("left_wheel_joint/position", session_->wheels.left);
                set_state("right_wheel_joint/position", session_->wheels.right);
            } catch (...) {
                joint_update_last_us_ =
                    (feedbot_chassis::steady_seconds() - joint_update_started) * 1e6;
                joint_update_max_us_ = std::max(joint_update_max_us_, joint_update_last_us_);
                throw;
            }
            joint_update_last_us_ = (feedbot_chassis::steady_seconds() - joint_update_started) * 1e6;
            joint_update_max_us_ = std::max(joint_update_max_us_, joint_update_last_us_);
            const auto now = feedbot_chassis::steady_seconds();
            if (now >= next_diagnostic_) {
                report();
                next_diagnostic_ = now + 0.2;
            }
            finish_read_timing(read_callback_started, true);
            return Return::OK;
        } catch (const std::exception &error) {
            finish_read_timing(read_callback_started, false);
            return fault(error);
        }
    }

    Return write(const rclcpp::Time &, const rclcpp::Duration &) override {
        if (!session_) return faulted_ ? Return::ERROR : Return::OK;
        try {
            session_->command(get_command<double>("left_wheel_joint/velocity"),
                              get_command<double>("right_wheel_joint/velocity"),
                              allow_charge_motion_);
            return Return::OK;
        } catch (const std::exception &error) { return fault(error); }
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
        return release() ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
    }
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
        const bool stopped = release();
        realtime_diagnostics_.reset();
        diagnostics_.reset();
        return stopped ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
    }
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
        return release() ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
    }
    CallbackReturn on_error(const rclcpp_lifecycle::State &) override {
        release();
        return CallbackReturn::ERROR;
    }
};
}
PLUGINLIB_EXPORT_CLASS(feedbot_chassis_ros2::ChassisSystem, hardware_interface::SystemInterface)
