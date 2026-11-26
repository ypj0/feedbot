#include "ltr2/core.hpp"
#include "ltr2/ethernet.hpp"
#include "ltr2/freshness.hpp"
#include "ltr2/passive_input.hpp"
#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

static void add_value(diagnostic_msgs::DiagnosticStatus& status,
                      const std::string& key, const std::string& value) {
    diagnostic_msgs::KeyValue entry;
    entry.key = key;
    entry.value = value;
    status.values.push_back(entry);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ltr2_passive_candidate", ros::init_options::AnonymousName);
    ros::NodeHandle node;
    ros::NodeHandle parameters("~");
    try {
        double duration = 15;
        std::string interface = "enp2s0";
        parameters.param("duration", duration, duration);
        parameters.param("interface", interface, interface);
        bool simulation = false;
        node.param("/use_sim_time", simulation, false);
        if (!std::isfinite(duration) || duration < 5 || duration > 60 || simulation) {
            throw std::runtime_error("requires wall-clock ROS and duration 5..60 seconds");
        }
        auto publisher = node.advertise<sensor_msgs::LaserScan>("/candidate/ltr2/scan", 2, false);
        auto diagnostics = node.advertise<diagnostic_msgs::DiagnosticArray>(
            "/candidate/ltr2/diagnostics", 2, false);
        if (publisher.getTopic() != "/candidate/ltr2/scan" ||
            diagnostics.getTopic() != "/candidate/ltr2/diagnostics") {
            throw std::runtime_error("candidate output remapping is forbidden");
        }
        ltr2::PassiveInput input(interface);
        ltr2::Assembler assembler;
        ltr2::Scan scan{};
        std::vector<uint8_t> packet;
        const double deadline = ltr2::monotonic_seconds() + duration;
        double last_scan = -1;
        double last_scan_age = 0;
        double last_diagnostic = -1;
        uint64_t errors = 0;
        uint64_t published = 0;
        while (ros::ok() && ltr2::monotonic_seconds() < deadline) {
            double received_wall = 0;
            if (input.receive(packet, received_wall)) {
                try {
                    const uint8_t* payload = nullptr;
                    size_t payload_length = 0;
                    if (ltr2::ethernet_payload(packet, payload, payload_length)) {
                        const double received_monotonic = ltr2::monotonic_seconds();
                        const double age = ltr2::wall_seconds() - received_wall;
                        if (!ltr2::acceptable_receive_age(age)) {
                            throw std::runtime_error("stale or future kernel receive time");
                        }
                        const auto block = ltr2::decode(payload, payload_length);
                        if (assembler.add(block, received_wall, received_monotonic, scan)) {
                            sensor_msgs::LaserScan message;
                            message.header.frame_id = "scan";
                            message.header.stamp.fromSec(scan.stamp);
                            message.angle_min = scan.angle_min;
                            message.angle_max = scan.angle_max;
                            message.angle_increment = scan.angle_increment;
                            message.time_increment = scan.time_increment;
                            message.scan_time = scan.scan_time;
                            message.range_min = 0.05;
                            message.range_max = 40.0;
                            message.ranges = scan.ranges;
                            message.intensities = scan.intensities;
                            if (!ltr2::publish_if_fresh(age, received_monotonic, received_wall,
                                    ltr2::monotonic_seconds, ltr2::wall_seconds,
                                    [&]() { publisher.publish(message); })) {
                                throw std::runtime_error("completed frame expired before publication");
                            }
                            last_scan = received_monotonic;
                            last_scan_age = age;
                            ++published;
                        }
                    }
                } catch (const std::exception& error) {
                    assembler.invalidate();
                    last_scan = -1;
                    ++errors;
                    ROS_WARN_THROTTLE(1.0, "Candidate rejects packet/frame: %s", error.what());
                }
            }
            const double now = ltr2::monotonic_seconds();
            if (now - last_diagnostic >= 0.1) {
                diagnostic_msgs::DiagnosticArray report;
                report.header.stamp = ros::Time::now();
                diagnostic_msgs::DiagnosticStatus status;
                const bool fresh = last_scan >= 0 && ltr2::fresh_input(last_scan_age, now - last_scan);
                status.name = "ltr2_candidate_data_only";
                status.hardware_id = "LT-R2-B profile at 192.168.10.160; not re-queried";
                status.level = fresh ? diagnostic_msgs::DiagnosticStatus::WARN :
                                       diagnostic_msgs::DiagnosticStatus::ERROR;
                status.message = fresh ? "Fresh complete frame; NOT navigation acceptance" :
                                         "No fresh complete frame; never replaying cached scan";
                add_value(status, "fresh_input", fresh ? "true" : "false");
                add_value(status, "navigation_accepted", "false");
                add_value(status, "published", std::to_string(published));
                add_value(status, "rejected", std::to_string(errors));
                add_value(status, "discarded", std::to_string(assembler.discarded()));
                add_value(status, "truncated_receives", std::to_string(input.truncated_receives()));
                add_value(status, "stamp_source", "kernel-reception estimate; not calibrated hardware time");
                report.status.push_back(status);
                diagnostics.publish(report);
                last_diagnostic = now;
            }
            ros::spinOnce();
        }
        ROS_INFO("Bounded passive candidate finished: published=%llu rejected=%llu",
                 static_cast<unsigned long long>(published), static_cast<unsigned long long>(errors));
        return published ? 0 : 2;
    } catch (const std::exception& error) {
        ROS_ERROR("Candidate stopped: %s", error.what());
        return 2;
    }
}
