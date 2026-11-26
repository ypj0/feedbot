#include "ltr2/core.hpp"
#include "ltr2/freshness.hpp"
#include "ltr2/lifecycle.hpp"
#include "ltr2/sdk_control.hpp"
#include "ltr2/udp_input.hpp"
#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Trigger.h>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <memory>
#include <thread>

static double monotonic_seconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

static double wall_seconds() {
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

static const char* state_name(ltr2::State state) {
    switch (state) {
        case ltr2::State::Disconnected: return "disconnected";
        case ltr2::State::Inactive: return "inactive";
        case ltr2::State::Starting: return "starting";
        case ltr2::State::Active: return "active";
        case ltr2::State::Sleeping: return "sleeping";
        case ltr2::State::Fault: return "fault";
    }
    return "fault";
}

class ManagedDriver {
public:
    ManagedDriver() : parameters_("~") {
        parameters_.param<std::string>("owner_directory", directory_, "/run/feedbot-lidar");
        parameters_.param<std::string>("owner_token", token_, "");
        scan_publisher_ = node_.advertise<sensor_msgs::LaserScan>("/candidate/ltr2/managed_scan", 2, false);
        diagnostics_ = node_.advertise<diagnostic_msgs::DiagnosticArray>("/candidate/ltr2/managed_diagnostics", 2, false);
        configure_service_ = node_.advertiseService("/ltr2_driver/configure", &ManagedDriver::configure, this);
        activate_service_ = node_.advertiseService("/ltr2_driver/activate", &ManagedDriver::activate, this);
        deactivate_service_ = node_.advertiseService("/ltr2_driver/deactivate", &ManagedDriver::deactivate, this);
        cleanup_service_ = node_.advertiseService("/ltr2_driver/cleanup", &ManagedDriver::cleanup, this);
    }

    ~ManagedDriver() {
        clear();
    }

    void poll() {
        try {
            if (lifecycle_) {
                if (!permitted()) {
                    operational_ = false;
                    lifecycle_->ownership_lost();
                    input_.reset();
                    assembler_ = ltr2::Assembler();
                } else if (lifecycle_->state() != ltr2::State::Fault) {
                    if (monotonic_seconds() - last_control_check_ >= 1.0) {
                        control_->validate();
                        last_control_check_ = monotonic_seconds();
                    }
                    lifecycle_->tick(monotonic_seconds());
                    if (lifecycle_->state() == ltr2::State::Active) {
                        for (unsigned count = 0; count < 32 && receive_one(true); ++count) {}
                    }
                }
            }
            status();
        } catch (const std::exception& error) {
            operational_ = false;
            if (lifecycle_) lifecycle_->transport_lost();
            ROS_ERROR_THROTTLE(1.0, "LT-R2 driver fault: %s", error.what());
        }
    }

private:
    bool permitted() const {
        if (token_.size() != 48 || token_.find_first_not_of("0123456789abcdef") != std::string::npos) return false;
        std::ifstream permit(directory_ + "/permit");
        std::string value;
        return static_cast<bool>(std::getline(permit, value)) && value == token_;
    }

    void require_permission() const {
        if (!permitted()) throw std::runtime_error("supervisor ownership permit absent or revoked");
    }

    int port_parameter(const std::string& name) {
        int value = 2105;
        parameters_.param(name, value, value);
        if (value < 1 || value > 65535) throw std::runtime_error("invalid " + name);
        return value;
    }

    bool configure(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
        if (lifecycle_) {
            response.message = "cleanup is required before configuring again";
            return true;
        }
        try {
            require_permission();
            bool simulated_time = false;
            node_.param("/use_sim_time", simulated_time, false);
            if (simulated_time) throw std::runtime_error("hardware driver requires wall time");
            const int candidate = open((directory_ + "/hardware.lock").c_str(), O_CREAT | O_RDWR, 0600);
            if (candidate < 0) throw std::runtime_error("cannot open ownership lock");
            if (flock(candidate, LOCK_EX | LOCK_NB)) {
                close(candidate);
                throw std::runtime_error("another managed driver owns the lock");
            }
            lock_ = candidate;
            std::string address, target, binding;
            parameters_.param<std::string>("device_address", address, "192.168.10.160");
            parameters_.param<std::string>("target_address", target, "255.255.255.255");
            parameters_.param<std::string>("bind_address", binding, "0.0.0.0");
            const int control_port = port_parameter("control_port");
            const int data_port = port_parameter("data_port");
            const int source_port = port_parameter("source_port");
            double timeout = 5.0;
            parameters_.param("startup_timeout", timeout, timeout);
            input_.reset(new ltr2::UdpInput(address, data_port, source_port, binding));
            control_.reset(new ltr2::SdkControl(address, target, control_port, data_port, "LT-R2-B"));
            lifecycle_.reset(new ltr2::Lifecycle(*control_, monotonic_seconds, timeout));
            lifecycle_->claim();
            require_permission();
            if (!lifecycle_->configure()) throw std::runtime_error(lifecycle_->last_error());
            last_control_check_ = monotonic_seconds();
            require_permission();
            assembler_ = ltr2::Assembler();
            status(true);
            response.success = true;
            response.message = "configured; explicit activation required";
        } catch (const std::exception& error) {
            response.message = error.what();
            clear();
        }
        return true;
    }

    bool activate(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
        try {
            require_permission();
            if (!lifecycle_ || !input_) throw std::runtime_error("driver is not configured");
            if (lifecycle_->state() != ltr2::State::Inactive && lifecycle_->state() != ltr2::State::Sleeping) {
                response.message = "activation requires inactive or sleeping state";
                return true;
            }
            assembler_ = ltr2::Assembler();
            operational_ = false;
            input_->drain();
            if (!lifecycle_->activate()) throw std::runtime_error(lifecycle_->last_error());
            activation_wall_ = wall_seconds();
            last_control_check_ = monotonic_seconds();
            status(true);
            while (ros::ok() && lifecycle_->state() == ltr2::State::Starting) {
                require_permission();
                receive_one(false);
                lifecycle_->tick(monotonic_seconds());
                status();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            require_permission();
            if (!ros::ok() || !lifecycle_->ready()) throw std::runtime_error("activation did not receive a fresh complete frame");
            operational_ = true;
            response.success = true;
            response.message = "active with fresh complete input; not navigation acceptance";
            status(true);
        } catch (const std::exception& error) {
            response.message = error.what();
            operational_ = false;
            if (lifecycle_) {
                if (permitted()) lifecycle_->transport_lost();
                else lifecycle_->ownership_lost();
            }
        }
        return true;
    }

    bool deactivate(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
        try {
            require_permission();
            if (!lifecycle_) throw std::runtime_error("driver is not configured");
            operational_ = false;
            status(true);
            response.success = lifecycle_->stop(true);
            assembler_ = ltr2::Assembler();
            response.message = response.success ? "streaming and measurement stopped" : "stop failed; physical stop not confirmed";
            status(true);
        } catch (const std::exception& error) {
            response.message = error.what();
        }
        return true;
    }

    bool cleanup(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
        response.success = clear();
        response.message = response.success ? "resources released" : "resources released; hardware stop not confirmed";
        return true;
    }

    bool clear() noexcept {
        bool stopped = true;
        operational_ = false;
        try { status(true); } catch (...) {}
        if (lifecycle_) {
            try {
                if (permitted() && lifecycle_->owned()) stopped = lifecycle_->release();
                else {
                    lifecycle_->ownership_lost();
                    stopped = false;
                }
            } catch (...) { stopped = false; }
        }
        input_.reset();
        lifecycle_.reset();
        control_.reset();
        assembler_ = ltr2::Assembler();
        try { status(true); } catch (...) {}
        if (lock_ >= 0) {
            flock(lock_, LOCK_UN);
            close(lock_);
            lock_ = -1;
        }
        return stopped;
    }

    bool receive_one(bool publish) {
        ltr2::Block block{};
        double received_wall = 0;
        if (!input_->receive(block, received_wall)) return false;
        const double received_monotonic = monotonic_seconds();
        const double age = wall_seconds() - received_wall;
        if (!ltr2::acceptable_receive_age(age) || received_wall < activation_wall_) {
            assembler_.invalidate();
            ++rejected_;
            return true;
        }
        ltr2::Scan scan{};
        if (!assembler_.add(block, received_wall, received_monotonic, scan)) return true;
        sensor_msgs::LaserScan message;
        message.header.frame_id = "scan";
        message.header.stamp.fromSec(scan.stamp);
        message.angle_min = scan.angle_min;
        message.angle_max = scan.angle_max;
        message.angle_increment = scan.angle_increment;
        message.scan_time = scan.scan_time;
        message.time_increment = scan.time_increment;
        message.range_min = 0.05;
        message.range_max = 40.0;
        message.ranges = scan.ranges;
        message.intensities = scan.intensities;
        const bool fresh = ltr2::publish_if_fresh(age, received_monotonic, received_wall,
            monotonic_seconds, wall_seconds, [&]() {
                require_permission();
                if (lifecycle_->frame(lifecycle_->epoch(), received_monotonic - std::max(0.0, age), monotonic_seconds()) && publish) {
                    scan_publisher_.publish(message);
                    ++scans_;
                }
            });
        if (!fresh) {
            assembler_.invalidate();
            ++rejected_;
        }
        return true;
    }

    void status(bool force = false) {
        const double now = monotonic_seconds();
        if (!force && now - last_status_ < 0.1) return;
        last_status_ = now;
        const bool ready = operational_ && lifecycle_ && lifecycle_->ready() && permitted();
        const bool healthy = lifecycle_ && lifecycle_->owned() && lifecycle_->state() != ltr2::State::Fault && permitted();
        const char* state = lifecycle_ ? state_name(lifecycle_->state()) : "disconnected";
        diagnostic_msgs::DiagnosticArray report;
        report.header.stamp = ros::Time::now();
        diagnostic_msgs::DiagnosticStatus entry;
        entry.name = "ltr2_managed_data";
        entry.hardware_id = "LT-R2-B";
        entry.level = healthy ? diagnostic_msgs::DiagnosticStatus::WARN : diagnostic_msgs::DiagnosticStatus::ERROR;
        entry.message = state;
        for (const auto& pair : std::vector<std::pair<std::string, std::string>>{
                {"ready", ready ? "true" : "false"}, {"healthy", healthy ? "true" : "false"},
                {"navigation_accepted", "false"}, {"scans", std::to_string(scans_)},
                {"rejected", std::to_string(rejected_)}, {"discarded_frames", std::to_string(assembler_.discarded())},
                {"malformed_packets", std::to_string(input_ ? input_->malformed_packets() : 0)},
                {"wrong_source_packets", std::to_string(input_ ? input_->discarded_packets() : 0)},
                {"stamp_source", "kernel reception estimate; not calibrated acquisition time"}}) {
            diagnostic_msgs::KeyValue value;
            value.key = pair.first;
            value.value = pair.second;
            entry.values.push_back(value);
        }
        report.status.push_back(entry);
        diagnostics_.publish(report);
        if (lock_ < 0) return;
        const std::string temporary = directory_ + "/status.tmp";
        std::ofstream output(temporary);
        output << std::setprecision(17) << "{\"ready\":" << (ready ? "true" : "false")
               << ",\"healthy\":" << (healthy ? "true" : "false") << ",\"heartbeat\":" << now
               << ",\"pid\":" << getpid() << ",\"scans\":" << scans_ << ",\"state\":\"" << state << "\"}\n";
        output.close();
        if (!output || std::rename(temporary.c_str(), (directory_ + "/status.json").c_str()))
            throw std::runtime_error("cannot publish supervisor status");
    }

    ros::NodeHandle node_;
    ros::NodeHandle parameters_;
    ros::Publisher scan_publisher_, diagnostics_;
    ros::ServiceServer configure_service_, activate_service_, deactivate_service_, cleanup_service_;
    std::string directory_, token_;
    int lock_ = -1;
    std::unique_ptr<ltr2::SdkControl> control_;
    std::unique_ptr<ltr2::Lifecycle> lifecycle_;
    std::unique_ptr<ltr2::UdpInput> input_;
    ltr2::Assembler assembler_;
    double activation_wall_ = 0;
    double last_status_ = 0;
    double last_control_check_ = 0;
    bool operational_ = false;
    uint64_t scans_ = 0;
    uint64_t rejected_ = 0;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ltr2_managed_driver");
    ManagedDriver driver;
    ros::WallRate rate(200);
    while (ros::ok()) {
        ros::spinOnce();
        driver.poll();
        rate.sleep();
    }
    return 0;
}
