#include "ltr2/core.hpp"
#include "ltr2/ethernet.hpp"
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>

static void write_array(const std::vector<float>& values) {
    std::cout << '[';
    for (size_t index = 0; index < values.size(); ++index) {
        if (index) {
            std::cout << ',';
        }
        if (std::isfinite(values[index])) {
            std::cout << values[index];
        } else {
            std::cout << "null";
        }
    }
    std::cout << ']';
}

int main(int argc, char** argv) {
    try {
        if (argc != 2) {
            throw std::runtime_error("usage: ltr2_replay CAPTURE.bin > scans.jsonl");
        }
        std::ifstream capture(argv[1], std::ios::binary);
        if (!capture) {
            throw std::runtime_error("capture open failed");
        }
        ltr2::Assembler assembler;
        ltr2::Scan scan{};
        uint64_t packets = 0;
        uint64_t scans = 0;
        uint64_t errors = 0;
        uint64_t records = 0;
        std::cout << std::setprecision(17);
        while (capture.peek() != EOF) {
            uint8_t header[12];
            capture.read(reinterpret_cast<char*>(header), sizeof(header));
            if (!capture || ++records > 1000000) {
                throw std::runtime_error("invalid capture header or record limit");
            }
            uint64_t timestamp_bits = 0;
            uint32_t length = 0;
            for (unsigned index = 0; index < 8; ++index) {
                timestamp_bits |= uint64_t(header[index]) << (index * 8);
            }
            for (unsigned index = 0; index < 4; ++index) {
                length |= uint32_t(header[index + 8]) << (index * 8);
            }
            double received = 0;
            static_assert(sizeof(received) == sizeof(timestamp_bits), "64-bit double required");
            std::memcpy(&received, &timestamp_bits, sizeof(received));
            if (!std::isfinite(received) || received <= 0 || length > 65535 || length < 14) {
                throw std::runtime_error("invalid capture record");
            }
            std::vector<uint8_t> packet(length);
            capture.read(reinterpret_cast<char*>(packet.data()), length);
            if (!capture) {
                throw std::runtime_error("truncated capture body");
            }
            try {
                const uint8_t* payload = nullptr;
                size_t payload_length = 0;
                if (!ltr2::ethernet_payload(packet, payload, payload_length)) {
                    continue;
                }
                const ltr2::Block block = ltr2::decode(payload, payload_length);
                ++packets;
                if (!assembler.add(block, received, received, scan)) {
                    continue;
                }
                ++scans;
                std::cout << "{\"mode\":\"replay_not_live\",\"frame\":" << scan.frame
                          << ",\"received\":" << received << ",\"stamp\":" << scan.stamp
                          << ",\"scan_time\":" << scan.scan_time
                          << ",\"time_increment\":" << scan.time_increment
                          << ",\"angle_min\":" << scan.angle_min
                          << ",\"angle_max\":" << scan.angle_max
                          << ",\"angle_increment\":" << scan.angle_increment
                          << ",\"ranges\":";
                write_array(scan.ranges);
                std::cout << ",\"intensities\":";
                write_array(scan.intensities);
                std::cout << "}\n";
            } catch (const std::exception&) {
                assembler.invalidate();
                ++errors;
            }
        }
        std::cerr << "{\"records\":" << records << ",\"valid_packets\":" << packets
                  << ",\"scans\":" << scans << ",\"errors\":" << errors
                  << ",\"discarded\":" << assembler.discarded()
                  << ",\"hardware_accepted\":false}\n";
        return scans && !errors ? 0 : 2;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 2;
    }
}
