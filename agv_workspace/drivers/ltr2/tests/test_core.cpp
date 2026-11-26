#include "ltr2/core.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr std::size_t packet_size = 1040U;

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

template <typename Callable>
void require_runtime_error(Callable callable, const std::string& message) {
    try {
        callable();
    } catch (const std::runtime_error&) {
        return;
    }
    throw std::runtime_error(message);
}

void put_u16(std::vector<uint8_t>& bytes, std::size_t offset, uint16_t value) {
    bytes[offset] = static_cast<uint8_t>(value & 0xffU);
    bytes[offset + 1U] = static_cast<uint8_t>(value >> 8U);
}

void put_u32(std::vector<uint8_t>& bytes, std::size_t offset, uint32_t value) {
    put_u16(bytes, offset, static_cast<uint16_t>(value & 0xffffU));
    put_u16(bytes, offset + 2U, static_cast<uint16_t>(value >> 16U));
}

uint16_t crc16(const std::vector<uint8_t>& bytes) {
    uint16_t crc = 0xffffU;
    for (std::size_t offset = 0; offset < bytes.size(); ++offset) {
        const uint8_t value = offset == 12U || offset == 13U ? 0U : bytes[offset];
        crc ^= static_cast<uint16_t>(value) << 8U;
        for (unsigned bit_index = 0; bit_index < 8U; ++bit_index) {
            crc = (crc & 0x8000U) != 0U
                      ? static_cast<uint16_t>((crc << 1U) ^ 0x1021U)
                      : static_cast<uint16_t>(crc << 1U);
        }
    }
    return crc;
}

void refresh_crc(std::vector<uint8_t>& bytes) {
    put_u16(bytes, 12U, crc16(bytes));
}

std::vector<uint8_t> make_packet(uint16_t frame, uint8_t index, uint32_t stamp) {
    std::vector<uint8_t> bytes(packet_size, 0U);
    put_u16(bytes, 0U, 0xffffU);
    put_u16(bytes, 2U, frame);
    bytes[4] = index;
    bytes[5] = 8U;
    put_u16(bytes, 6U, 256U);
    put_u32(bytes, 8U, stamp);
    put_u16(bytes, 14U, 0x0101U);
    for (std::size_t sample_index = 0; sample_index < 256U; ++sample_index) {
        put_u16(bytes, 16U + sample_index * 2U,
                static_cast<uint16_t>(index * 256U + sample_index + 1U));
        put_u16(bytes, 528U + sample_index * 2U,
                static_cast<uint16_t>(2000U + index * 256U + sample_index));
    }
    refresh_crc(bytes);
    return bytes;
}

ltr2::Block make_block(uint16_t frame, uint8_t index, uint32_t stamp) {
    ltr2::Block block{};
    block.frame = frame;
    block.index = index;
    block.stamp = stamp;
    for (std::size_t sample_index = 0; sample_index < block.ranges.size(); ++sample_index) {
        const std::size_t beam_index = static_cast<std::size_t>(index) * 256U + sample_index;
        block.ranges[sample_index] = static_cast<uint16_t>(beam_index + 1U);
        block.intensities[sample_index] = static_cast<uint16_t>(2000U + beam_index);
    }
    return block;
}

bool feed_frame(ltr2::Assembler& assembler, uint16_t frame, uint32_t first_stamp,
                double first_wall, double first_monotonic, uint32_t block_delta,
                double host_delta, ltr2::Scan& scan) {
    bool published = false;
    for (uint8_t block_index = 0U; block_index < 8U; ++block_index) {
        const ltr2::Block block = make_block(
            frame, block_index, first_stamp + static_cast<uint32_t>(block_index) * block_delta);
        const bool result = assembler.add(
            block, first_wall + static_cast<double>(block_index) * host_delta,
            first_monotonic + static_cast<double>(block_index) * host_delta, scan);
        require(!published, "frame published more than once");
        published = result;
    }
    return published;
}

void test_decode() {
    const std::vector<uint8_t> bytes = make_packet(0x1234U, 3U, 0x89abcdefU);
    const ltr2::Block block = ltr2::decode(bytes.data(), bytes.size());
    require(block.frame == 0x1234U, "decoded frame mismatch");
    require(block.index == 3U, "decoded index mismatch");
    require(block.stamp == 0x89abcdefU, "decoded timestamp mismatch");
    require(block.ranges.front() == 769U && block.ranges.back() == 1024U,
            "decoded ranges mismatch");
    require(block.intensities.front() == 2768U && block.intensities.back() == 3023U,
            "decoded intensities mismatch");

    require_runtime_error([&]() { ltr2::decode(nullptr, bytes.size()); }, "null packet accepted");
    for (std::size_t truncated_size = 0U; truncated_size < bytes.size(); ++truncated_size) {
        require_runtime_error(
            [&]() { ltr2::decode(bytes.data(), truncated_size); }, "truncated packet accepted");
    }
    std::vector<uint8_t> extended = bytes;
    extended.push_back(0U);
    require_runtime_error(
        [&]() { ltr2::decode(extended.data(), extended.size()); }, "extended packet accepted");

    for (std::size_t byte_index = 0U; byte_index < bytes.size(); ++byte_index) {
        for (unsigned bit_index = 0U; bit_index < 8U; ++bit_index) {
            std::vector<uint8_t> damaged = bytes;
            damaged[byte_index] ^= static_cast<uint8_t>(1U << bit_index);
            require_runtime_error(
                [&]() { ltr2::decode(damaged.data(), damaged.size()); },
                "single-bit corruption accepted");
        }
    }
}

void test_crc_valid_wrong_layout() {
    const std::array<std::pair<std::size_t, uint16_t>, 7> mutations{{
        {0U, 0xfffeU}, {4U, 8U}, {5U, 7U}, {5U, 0U},
        {6U, 255U}, {14U, 0x0100U}, {14U, 0x0001U},
    }};
    for (const auto& mutation : mutations) {
        std::vector<uint8_t> bytes = make_packet(1U, 0U, 1000U);
        if (mutation.first == 4U || mutation.first == 5U) {
            bytes[mutation.first] = static_cast<uint8_t>(mutation.second);
        } else {
            put_u16(bytes, mutation.first, mutation.second);
        }
        refresh_crc(bytes);
        require_runtime_error(
            [&]() { ltr2::decode(bytes.data(), bytes.size()); }, "CRC-valid wrong layout accepted");
    }
}

void test_geometry_and_conversion() {
    ltr2::Assembler assembler;
    ltr2::Scan scan{};
    for (uint8_t block_index = 0U; block_index < 8U; ++block_index) {
        ltr2::Block block = make_block(77U, block_index, 100000U + block_index * 8000U);
        if (block_index == 1U) {
            block.ranges[231U] = 0U;
            block.intensities[231U] = 65535U;
        }
        const bool published = assembler.add(
            block, 1000.0 + block_index * 0.008, 50.0 + block_index * 0.008, scan);
        require(published == (block_index == 7U), "unexpected publication position");
    }

    const double pi = 3.1415926535897932384626433832795;
    const double expected_increment = 2.0 * pi / 2048.0;
    require(scan.frame == 77U, "scan frame mismatch");
    require(scan.ranges.size() == 1075U && scan.intensities.size() == 1075U,
            "crop sample count mismatch");
    require(std::isinf(scan.ranges.front()), "zero range did not become infinity");
    require(scan.intensities.front() == 0.0F, "zero range intensity was not cleared");
    require(std::abs(scan.ranges[1] - 0.978F) < 1.0e-6F, "range scale mismatch");
    require(scan.intensities[1] == 2488.0F, "intensity conversion mismatch");
    require(std::abs(scan.scan_time - 0.064) < 1.0e-12, "scan time mismatch");
    require(std::abs(scan.time_increment - 0.064 / 2048.0) < 1.0e-15,
            "time increment mismatch");
    require(std::abs(scan.angle_increment - expected_increment) < 1.0e-15,
            "angle increment mismatch");
    require(std::abs(scan.angle_min + 537.0 * expected_increment) < 1.0e-15,
            "angle minimum mismatch");
    require(std::abs(scan.angle_max - 537.0 * expected_increment) < 1.0e-15,
            "angle maximum mismatch");
    require(std::abs(scan.stamp - (1000.0 - 0.008 + 487.0 * 0.064 / 2048.0)) < 1.0e-12,
            "host stamp estimator mismatch");
}

void test_sequence_failures_and_recovery() {
    {
        ltr2::Assembler assembler;
        ltr2::Scan scan{};
        require(!assembler.add(make_block(5U, 1U, 9000U), 1.0, 1.0, scan),
                "startup nonzero block published");
        require(!feed_frame(assembler, 5U, 10000U, 2.0, 2.0, 8000U, 0.008, scan),
                "out-of-order startup frame restarted");
        require(feed_frame(assembler, 6U, 100000U, 3.0, 3.0, 8000U, 0.008, scan),
                "startup out-of-order recovery failed");
    }
    {
        ltr2::Assembler assembler;
        ltr2::Scan scan{};
        require(!assembler.add(make_block(10U, 0U, 1000U), 1.0, 1.0, scan),
                "initial block published");
        require(!assembler.add(make_block(10U, 0U, 2000U), 1.001, 1.001, scan),
                "duplicate block published");
        require(assembler.discarded() == 1U, "duplicate did not discard partial frame");
        require(feed_frame(assembler, 11U, 10000U, 2.0, 2.0, 8000U, 0.008, scan),
                "duplicate recovery failed");
    }
    {
        ltr2::Assembler assembler;
        ltr2::Scan scan{};
        assembler.add(make_block(20U, 0U, 1000U), 1.0, 1.0, scan);
        require(!assembler.add(make_block(20U, 2U, 17000U), 1.016, 1.016, scan),
                "out-of-order block published");
        require(feed_frame(assembler, 21U, 20000U, 2.0, 2.0, 8000U, 0.008, scan),
                "out-of-order recovery failed");
    }
    {
        ltr2::Assembler assembler;
        ltr2::Scan scan{};
        ltr2::Block invalid_index = make_block(25U, 0U, 1000U);
        invalid_index.index = 8U;
        require(!assembler.add(invalid_index, 1.0, 1.0, scan),
                "direct invalid block index accepted");
        require(feed_frame(assembler, 25U, 10000U, 2.0, 2.0, 8000U, 0.008, scan),
                "direct invalid block recovery failed");
    }
    {
        ltr2::Assembler assembler;
        ltr2::Scan scan{};
        assembler.add(make_block(30U, 0U, 1000U), 1.0, 1.0, scan);
        require(!assembler.add(make_block(31U, 1U, 9000U), 1.008, 1.008, scan),
                "mixed frame published");
        require(!feed_frame(assembler, 31U, 20000U, 2.0, 2.0, 8000U, 0.008, scan),
                "previously observed mixed frame restarted");
        require(feed_frame(assembler, 32U, 100000U, 3.0, 3.0, 8000U, 0.008, scan),
                "mixed-frame recovery failed");
    }
    {
        ltr2::Assembler assembler;
        ltr2::Scan scan{};
        assembler.add(make_block(40U, 0U, 1000U), 1.0, 1.0, scan);
        assembler.add(make_block(40U, 1U, 9000U), 1.008, 1.008, scan);
        require(!assembler.add(make_block(41U, 0U, 70000U), 2.0, 2.0, scan),
                "replacement frame published early");
        for (uint8_t block_index = 1U; block_index < 8U; ++block_index) {
            const bool published = assembler.add(
                make_block(41U, block_index, 70000U + block_index * 8000U),
                2.0 + block_index * 0.008, 2.0 + block_index * 0.008, scan);
            require(published == (block_index == 7U), "dropped-frame recovery failed");
        }
    }
}

void test_clock_validation() {
    {
        ltr2::Assembler assembler;
        ltr2::Scan scan{};
        require(!assembler.add(make_block(45U, 0U, 1000U), 0.0, 1.0, scan),
                "nonpositive wall clock accepted");
        require(feed_frame(assembler, 45U, 10000U, 1.0, 2.0, 8000U, 0.008, scan),
                "nonpositive wall recovery failed");
    }
    {
        ltr2::Assembler assembler;
        ltr2::Scan scan{};
        assembler.add(make_block(50U, 0U, 1000U), 1.0, 1.0, scan);
        require(!assembler.add(make_block(50U, 1U, 9000U),
                               std::numeric_limits<double>::quiet_NaN(), 1.008, scan),
                "non-finite wall clock accepted");
        require(feed_frame(assembler, 51U, 10000U, 2.0, 2.0, 8000U, 0.008, scan),
                "non-finite wall recovery failed");
    }
    {
        ltr2::Assembler assembler;
        ltr2::Scan scan{};
        assembler.add(make_block(60U, 0U, 1000U), 10.0, 10.0, scan);
        require(!assembler.add(make_block(60U, 1U, 9000U), 9.0, 10.008, scan),
                "wall clock regression accepted");
        require(feed_frame(assembler, 61U, 10000U, 9.1, 11.0, 8000U, 0.008, scan),
                "wall regression recovery failed");
    }
    {
        ltr2::Assembler assembler;
        ltr2::Scan scan{};
        assembler.add(make_block(70U, 0U, 1000U), 10.0, 10.0, scan);
        require(!assembler.add(make_block(70U, 1U, 9000U), 10.008, 9.0, scan),
                "monotonic clock regression accepted");
        require(feed_frame(assembler, 71U, 10000U, 11.0, 9.1, 8000U, 0.008, scan),
                "monotonic regression recovery failed");
    }
    {
        ltr2::Assembler assembler;
        ltr2::Scan scan{};
        require(feed_frame(assembler, 75U, 100000U, 100.0, 100.0, 8000U, 0.008, scan),
                "stamp monotonicity baseline failed");
        const double baseline_stamp = scan.stamp;
        require(!assembler.add(make_block(76U, 0U, 164000U), 10.0, 101.0, scan),
                "wall rollback packet published");
        require(!feed_frame(assembler, 76U, 164000U, 11.0, 102.0, 8000U, 0.008, scan),
                "older estimated stamp published");
        require(scan.stamp == baseline_stamp, "failed frame modified prior scan");
        require(feed_frame(assembler, 77U, 228000U, 101.0, 103.0, 8000U, 0.008, scan),
                "stamp monotonicity recovery failed");
        require(scan.stamp > baseline_stamp, "emitted stamp did not advance");
    }
}

void test_timing_limits_and_wraps() {
    {
        ltr2::Assembler assembler;
        ltr2::Scan scan{};
        require(!feed_frame(assembler, 80U, 1000U, 1.0, 1.0, 3000U, 0.003, scan),
                "short scan period accepted");
        require(feed_frame(assembler, 81U, 100000U, 2.0, 2.0, 8000U, 0.008, scan),
                "short-period recovery failed");
    }
    {
        ltr2::Assembler assembler;
        ltr2::Scan scan{};
        assembler.add(make_block(85U, 0U, 10000U), 1.0, 1.0, scan);
        require(!assembler.add(make_block(85U, 1U, 10000U), 1.008, 1.008, scan),
                "non-increasing device timestamp accepted");
        require(feed_frame(assembler, 86U, 100000U, 2.0, 2.0, 8000U, 0.008, scan),
                "device timestamp recovery failed");
    }
    {
        ltr2::Assembler assembler;
        ltr2::Scan scan{};
        require(!feed_frame(assembler, 90U, 1000U, 1.0, 1.0, 19000U, 0.019, scan),
                "long scan period accepted");
        require(feed_frame(assembler, 91U, 200000U, 2.0, 2.0, 8000U, 0.008, scan),
                "long-period recovery failed");
    }
    {
        ltr2::Assembler assembler;
        ltr2::Scan scan{};
        require(!feed_frame(assembler, 100U, 1000U, 1.0, 1.0, 8000U, 0.031, scan),
                "overlong host frame accepted");
        require(feed_frame(assembler, 101U, 100000U, 2.0, 2.0, 8000U, 0.008, scan),
                "host-duration recovery failed");
    }
    {
        ltr2::Assembler assembler;
        ltr2::Scan scan{};
        require(feed_frame(assembler, 65535U, 0xffff7000U, 1.0, 1.0, 8000U, 0.008, scan),
                "device counter wrap failed");
        require(feed_frame(assembler, 0U, 100000U, 2.0, 2.0, 8000U, 0.008, scan),
                "frame counter wrap failed");
        assembler.invalidate();
        require(!feed_frame(assembler, 0U, 200000U, 3.0, 3.0, 8000U, 0.008, scan),
                "completed frame replay accepted after invalidation");
        require(feed_frame(assembler, 1U, 300000U, 4.0, 4.0, 8000U, 0.008, scan),
                "post-replay recovery failed");
    }
    {
        ltr2::Assembler assembler;
        ltr2::Scan scan{};
        require(feed_frame(assembler, 200U, 1000U, 1.0, 1.0, 8000U, 0.008, scan),
                "baseline frame failed");
        require(!feed_frame(assembler, 5000U, 100000U, 2.0, 2.0, 8000U, 0.008, scan),
                "unbounded frame advance accepted");
        require(feed_frame(assembler, 201U, 200000U, 3.0, 3.0, 8000U, 0.008, scan),
                "bounded sequence recovery failed");
    }
    {
        ltr2::Assembler assembler;
        ltr2::Scan scan{};
        require(feed_frame(assembler, 300U, 1000000U, 1.0, 1.0, 8000U, 0.008, scan),
                "device continuity baseline failed");
        require(!assembler.add(make_block(301U, 0U, 500000U), 2.0, 2.0, scan),
                "cross-frame device timestamp rollback accepted");
        require(!assembler.add(make_block(302U, 0U, 600000U), 3.0, 3.0, scan),
                "device rollback recovered without restart");
    }
}

uint16_t network_u16(const uint8_t* data) {
    return static_cast<uint16_t>(static_cast<uint16_t>(data[0]) << 8U) |
           static_cast<uint16_t>(data[1]);
}

void test_archived_capture(const std::string& path) {
    std::ifstream input(path, std::ios::binary);
    require(input.good(), "cannot open archived raw capture");
    std::size_t packet_count = 0U;
    std::size_t decoded_count = 0U;
    std::size_t scan_count = 0U;
    double first_wall = 0.0;
    ltr2::Assembler assembler;
    ltr2::Scan scan{};
    while (true) {
        double receive_wall = 0.0;
        uint32_t frame_size = 0U;
        input.read(reinterpret_cast<char*>(&receive_wall), sizeof(receive_wall));
        if (input.eof()) {
            break;
        }
        require(input.good(), "truncated archived record timestamp");
        input.read(reinterpret_cast<char*>(&frame_size), sizeof(frame_size));
        require(input.good(), "truncated archived record length");
        require(frame_size >= 42U && frame_size <= 65535U, "invalid archived Ethernet length");
        std::vector<uint8_t> ethernet(frame_size);
        input.read(reinterpret_cast<char*>(ethernet.data()), frame_size);
        require(input.good(), "truncated archived Ethernet frame");
        require(network_u16(ethernet.data() + 12U) == 0x0800U, "archived frame is not IPv4");
        const std::size_t ip_header_size = static_cast<std::size_t>(ethernet[14U] & 0x0fU) * 4U;
        require(ip_header_size >= 20U, "archived IPv4 header is invalid");
        ++packet_count;
        if (ethernet[23U] != 17U) {
            continue;
        }
        const std::size_t udp_offset = 14U + ip_header_size;
        require(udp_offset + 8U <= ethernet.size(), "archived UDP header truncated");
        const std::size_t udp_size = network_u16(ethernet.data() + udp_offset + 4U);
        if (udp_size != packet_size + 8U) {
            continue;
        }
        require(udp_size == packet_size + 8U && udp_offset + udp_size <= ethernet.size(),
                "archived UDP payload length mismatch");
        const ltr2::Block block = ltr2::decode(ethernet.data() + udp_offset + 8U, packet_size);
        if (decoded_count == 0U) {
            first_wall = receive_wall;
        }
        if (assembler.add(block, receive_wall, receive_wall - first_wall, scan)) {
            require(scan.ranges.size() == 1075U, "archived scan geometry mismatch");
            ++scan_count;
        }
        ++decoded_count;
    }
    require(packet_count >= 1000U, "too few archived Ethernet records");
    require(decoded_count >= 1000U, "too few archived OOB packets");
    require(scan_count >= 180U, "too few archived complete scans");
}

}

int main(int argc, char** argv) {
    try {
        test_decode();
        test_crc_valid_wrong_layout();
        test_geometry_and_conversion();
        test_sequence_failures_and_recovery();
        test_clock_validation();
        test_timing_limits_and_wraps();
        if (argc == 2) {
            test_archived_capture(argv[1]);
        } else {
            require(argc == 1, "usage: test_core [archived_raw_capture]");
        }
        std::cout << "ltr2 core tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "ltr2 core tests failed: " << error.what() << '\n';
        return 1;
    }
}
