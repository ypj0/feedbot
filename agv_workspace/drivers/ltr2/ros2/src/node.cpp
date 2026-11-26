#include "ltr2/freshness.hpp"
#include "ltr2/lifecycle.hpp"
#include "ltr2/sdk_control.hpp"
#include "ltr2/udp_input.hpp"
#include "ltr2_ros2/message.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <memory>
#include <thread>

using Callback = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

static double monotonic_seconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

class Driver final : public rclcpp_lifecycle::LifecycleNode {
public:
    Driver() : LifecycleNode("ltr2_driver") {
        declare_parameter("owner_directory", "/run/feedbot-lidar");
        declare_parameter("owner_token", "");
        scans_ = rclcpp::create_publisher<sensor_msgs::msg::LaserScan>(
            this, "/scan", rclcpp::SensorDataQoS());
        diagnostics_ = rclcpp::create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            this, "/diagnostics", rclcpp::QoS(10));
        diagnostic_output_ = std::make_unique<DiagnosticOutput>(diagnostics_);
        timer_ = create_wall_timer(std::chrono::milliseconds(5), [this] { poll(); });
    }

    ~Driver() override {
        if (!clear()) RCLCPP_ERROR(get_logger(), "cleanup failed; hardware stop not confirmed");
    }

    Callback on_configure(const rclcpp_lifecycle::State&) override {
        try {
            directory_ = get_parameter("owner_directory").as_string();
            token_ = get_parameter("owner_token").as_string();
            require_permission();
            if (get_parameter("use_sim_time").as_bool())
                throw std::runtime_error("hardware driver requires system time");
            const int candidate = open((directory_ + "/hardware.lock").c_str(), O_CREAT | O_RDWR, 0600);
            if (candidate < 0) throw std::runtime_error("cannot open ownership lock");
            if (flock(candidate, LOCK_EX | LOCK_NB)) {
                close(candidate);
                throw std::runtime_error("managed driver lock busy");
            }
            lock_ = candidate;
            control_ = std::make_unique<ltr2::SdkControl>("192.168.10.160", "255.255.255.255");
            lifecycle_ = std::make_unique<ltr2::Lifecycle>(*control_, monotonic_seconds);
            lifecycle_->claim();
            if (!lifecycle_->configure()) throw std::runtime_error(lifecycle_->last_error());
            status(true);
            return Callback::SUCCESS;
        } catch (const std::exception& error) {
            RCLCPP_ERROR(get_logger(), "configure: %s", error.what());
            if (!clear()) RCLCPP_ERROR(get_logger(), "configure cleanup failed");
            return Callback::FAILURE;
        }
    }

    Callback on_activate(const rclcpp_lifecycle::State&) override {
        try {
            require_permission();
            if (!lifecycle_) throw std::runtime_error("driver not configured");
            control_->validate();
            assembler_ = ltr2::Assembler();
            input_ = std::make_unique<ltr2::UdpInput>("192.168.10.160");
            input_->drain();
            if (!lifecycle_->activate()) throw std::runtime_error(lifecycle_->last_error());
            activation_wall_ = system_clock_.now().seconds();
            while (rclcpp::ok() && lifecycle_->state() == ltr2::State::Starting) {
                require_permission();
                receive_one(false);
                lifecycle_->tick(monotonic_seconds());
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            if (!rclcpp::ok() || !lifecycle_->ready())
                throw std::runtime_error("activation failed: " + lifecycle_->last_error());
            scan_output_ = std::make_unique<ScanOutput>(scans_);
            operational_ = true;
            status(true);
            return Callback::SUCCESS;
        } catch (const std::exception& error) {
            RCLCPP_ERROR(get_logger(), "activate: %s", error.what());
            operational_ = false;
            if (lifecycle_) {
                if (!permitted()) lifecycle_->ownership_lost();
                else if (lifecycle_->owned() && !lifecycle_->stop(true))
                    RCLCPP_ERROR(get_logger(), "activation stop failed");
            }
            release_input();
            scan_output_.reset();
            return Callback::FAILURE;
        }
    }

    Callback on_deactivate(const rclcpp_lifecycle::State&) override {
        operational_ = false;
        bool stopped = false;
        try {
            require_permission();
            stopped = lifecycle_ && lifecycle_->stop(true);
        } catch (const std::exception& error) {
            RCLCPP_ERROR(get_logger(), "stop: %s", error.what());
        }
        release_input();
        scan_output_.reset();
        status(true);
        if (!stopped) RCLCPP_ERROR(get_logger(), "stop failed; hardware stop not confirmed");
        return stopped ? Callback::SUCCESS : Callback::ERROR;
    }

    Callback on_cleanup(const rclcpp_lifecycle::State&) override {
        return clear() ? Callback::SUCCESS : Callback::ERROR;
    }

    Callback on_shutdown(const rclcpp_lifecycle::State&) override {
        return clear() ? Callback::SUCCESS : Callback::ERROR;
    }

    Callback on_error(const rclcpp_lifecycle::State&) override {
        clear();
        return Callback::FAILURE;
    }

private:
    bool permitted() const {
        std::ifstream permit(directory_ + "/permit");
        std::string value;
        return !token_.empty() && static_cast<bool>(std::getline(permit, value)) && value == token_;
    }

    void require_permission() const {
        if (!permitted()) throw std::runtime_error("supervisor ownership permit absent or revoked");
    }

    void release_input() {
        if (input_) malformed_ += input_->malformed_packets();
        input_.reset();
        discarded_ += assembler_.discarded();
        assembler_ = ltr2::Assembler();
    }

    bool clear() {
        operational_ = false;
        bool stopped = true;
        if (lifecycle_) {
            if (permitted() && lifecycle_->owned()) stopped = lifecycle_->release();
            else {
                lifecycle_->ownership_lost();
                stopped = false;
            }
        }
        release_input();
        scan_output_.reset();
        lifecycle_.reset();
        control_.reset();
        if (lock_ >= 0) {
            flock(lock_, LOCK_UN);
            close(lock_);
            lock_ = -1;
        }
        if (!stopped) RCLCPP_ERROR(get_logger(), "cleanup released resources; hardware stop unconfirmed");
        return stopped;
    }

    bool receive_one(bool publish) {
        ltr2::Block block{};
        double received_wall = 0;
        if (!input_->receive(block, received_wall)) return false;
        const double received_monotonic = monotonic_seconds();
        const double age = system_clock_.now().seconds() - received_wall;
        if (!ltr2::acceptable_receive_age(age) || received_wall < activation_wall_) {
            assembler_.invalidate();
            ++rejected_;
            return true;
        }
        ltr2::Scan scan{};
        if (!assembler_.add(block, received_wall, received_monotonic, scan)) return true;
        const auto complete_frame_received = system_clock_.now();
        auto message = ltr2_ros2::to_message(scan, complete_frame_received);
        const bool fresh = ltr2::publish_if_fresh(age, received_monotonic, received_wall,
            monotonic_seconds, [this] { return system_clock_.now().seconds(); }, [&] {
                require_permission();
                if (lifecycle_->frame(lifecycle_->epoch(), received_monotonic - std::max(0.0, age),
                                      monotonic_seconds())) {
                    ++received_frames_;
                    if (publish) {
                        if (scan_output_->try_publish(message)) ++scan_handoffs_;
                        else ++scan_handoff_drops_;
                    }
                }
            });
        if (!fresh) {
            assembler_.invalidate();
            ++rejected_;
        }
        return true;
    }

    void poll() {
        if (lifecycle_) {
            require_permission();
            if (operational_) {
                for (unsigned count = 0; count < 32 && receive_one(true); ++count) {}
                lifecycle_->tick(monotonic_seconds());
                if (!lifecycle_->ready()) throw std::runtime_error(lifecycle_->last_error());
            }
        }
        status();
    }

    void status(bool force = false) {
        const double now = monotonic_seconds();
        const double elapsed = now - last_status_;
        if (!force && elapsed < 0.1) return;
        if (now - last_rate_time_ >= 1.0) {
            rate_ = last_rate_time_ > 0 ? (received_frames_ - last_received_frames_) / (now - last_rate_time_) : 0;
            last_rate_time_ = now;
            last_received_frames_ = received_frames_;
        }
        last_status_ = now;
        const bool healthy = lifecycle_ && lifecycle_->owned() &&
            lifecycle_->state() != ltr2::State::Fault && permitted();
        diagnostic_msgs::msg::DiagnosticArray report;
        report.header.stamp = system_clock_.now();
        diagnostic_msgs::msg::DiagnosticStatus entry;
        entry.name = "ltr2_ros2";
        entry.hardware_id = "LT-R2-B";
        entry.level = healthy ? entry.WARN : entry.ERROR;
        entry.message = "绝对时间对齐未验证";
        for (const auto& pair : std::vector<std::pair<std::string, std::string>>{
                {"state", operational_ ? "active" : (lifecycle_ ? "inactive" : "unconfigured")},
                {"frame_rate_hz", std::to_string(rate_)},
                {"received_frames", std::to_string(received_frames_)},
                {"scan_handoff_count", std::to_string(scan_handoffs_)},
                {"scan_handoff_drop_count", std::to_string(scan_handoff_drops_)},
                {"diagnostic_handoff_drop_count", std::to_string(diagnostic_handoff_drops_)},
                {"publication_method", force && !operational_ ? "synchronous_inactive_status" :
                    "realtime_try_handoff_not_dds_completion"},
                {"discarded_frames", std::to_string(discarded_ + assembler_.discarded())},
                {"rejected_stale", std::to_string(rejected_)},
                {"malformed_packets", std::to_string(malformed_ + (input_ ? input_->malformed_packets() : 0))},
                {"stamp_source", "complete_frame_receive_system_time"},
                {"absolute_time_alignment_verified", "false"}}) {
            diagnostic_msgs::msg::KeyValue value;
            value.key = pair.first;
            value.value = pair.second;
            entry.values.push_back(value);
        }
        report.status.push_back(entry);
        if (force && !operational_) diagnostics_->publish(report);
        else if (!diagnostic_output_->try_publish(report)) ++diagnostic_handoff_drops_;
        if (lock_ < 0) return;
        std::ofstream output(directory_ + "/status.tmp");
        output << std::setprecision(17) << "{\"healthy\":" << (healthy ? "true" : "false")
               << ",\"ready\":" << (operational_ && healthy ? "true" : "false")
               << ",\"heartbeat\":" << now << ",\"pid\":" << getpid() << "}\n";
        output.close();
        if (!output || std::rename((directory_ + "/status.tmp").c_str(), (directory_ + "/status.json").c_str()))
            throw std::runtime_error("cannot publish supervisor status");
    }

    rclcpp::Clock system_clock_{RCL_SYSTEM_TIME};
    using ScanOutput = realtime_tools::RealtimePublisher<sensor_msgs::msg::LaserScan>;
    using DiagnosticOutput = realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scans_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
    std::unique_ptr<ScanOutput> scan_output_;
    std::unique_ptr<DiagnosticOutput> diagnostic_output_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<ltr2::SdkControl> control_;
    std::unique_ptr<ltr2::Lifecycle> lifecycle_;
    std::unique_ptr<ltr2::UdpInput> input_;
    ltr2::Assembler assembler_;
    std::string directory_, token_;
    int lock_ = -1;
    bool operational_ = false;
    double activation_wall_ = 0, last_status_ = 0;
    double last_rate_time_ = 0, rate_ = 0;
    uint64_t received_frames_ = 0, last_received_frames_ = 0, scan_handoffs_ = 0;
    uint64_t scan_handoff_drops_ = 0, diagnostic_handoff_drops_ = 0;
    uint64_t discarded_ = 0, rejected_ = 0, malformed_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int result = 0;
    std::shared_ptr<Driver> node;
    try {
        node = std::make_shared<Driver>();
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node->get_node_base_interface());
        executor.spin();
    } catch (const std::exception& error) {
        RCLCPP_ERROR(rclcpp::get_logger("ltr2_driver"), "%s", error.what());
        result = 1;
    }
    node.reset();
    rclcpp::shutdown();
    return result;
}
