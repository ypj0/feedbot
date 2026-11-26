#include "ltr2/udp_input.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
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
    for (std::size_t offset = 0U; offset < bytes.size(); ++offset) {
        const uint8_t value = offset == 12U || offset == 13U ? 0U : bytes[offset];
        crc ^= static_cast<uint16_t>(value) << 8U;
        for (unsigned bit_index = 0U; bit_index < 8U; ++bit_index) {
            crc = (crc & 0x8000U) != 0U
                      ? static_cast<uint16_t>((crc << 1U) ^ 0x1021U)
                      : static_cast<uint16_t>(crc << 1U);
        }
    }
    return crc;
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
    for (std::size_t sample_index = 0U; sample_index < 256U; ++sample_index) {
        put_u16(bytes, 16U + sample_index * 2U,
                static_cast<uint16_t>(sample_index + 1U));
        put_u16(bytes, 528U + sample_index * 2U,
                static_cast<uint16_t>(2000U + sample_index));
    }
    put_u16(bytes, 12U, crc16(bytes));
    return bytes;
}

class Sender {
public:
    explicit Sender(const std::string& bind_ip, uint16_t port = 0U) {
        descriptor_ = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
        if (descriptor_ < 0) {
            throw std::runtime_error("sender socket");
        }
        sockaddr_in local{};
        local.sin_family = AF_INET;
        local.sin_port = htons(port);
        require(inet_pton(AF_INET, bind_ip.c_str(), &local.sin_addr) == 1,
                "sender bind address");
        if (bind(descriptor_, reinterpret_cast<const sockaddr*>(&local), sizeof(local)) != 0) {
            const std::string message = std::string("sender bind: ") + std::strerror(errno);
            close(descriptor_);
            descriptor_ = -1;
            throw std::runtime_error(message);
        }
        socklen_t size = sizeof(local);
        require(getsockname(descriptor_, reinterpret_cast<sockaddr*>(&local), &size) == 0,
                "sender endpoint");
        port_ = ntohs(local.sin_port);
    }

    ~Sender() {
        if (descriptor_ >= 0) {
            close(descriptor_);
        }
    }

    Sender(const Sender&) = delete;
    Sender& operator=(const Sender&) = delete;

    uint16_t port() const {
        return port_;
    }

    void send_to(const std::vector<uint8_t>& bytes, uint16_t destination_port) const {
        sockaddr_in destination{};
        destination.sin_family = AF_INET;
        destination.sin_port = htons(destination_port);
        require(inet_pton(AF_INET, "127.0.0.1", &destination.sin_addr) == 1,
                "destination address");
        const ssize_t sent = sendto(descriptor_, bytes.data(), bytes.size(), 0,
                                    reinterpret_cast<const sockaddr*>(&destination),
                                    sizeof(destination));
        require(sent == static_cast<ssize_t>(bytes.size()), "datagram send failed");
    }

private:
    int descriptor_ = -1;
    uint16_t port_ = 0U;
};

void test_valid_packet_and_timestamp() {
    Sender sender("127.0.0.1");
    ltr2::UdpInput input("127.0.0.1", 0U, sender.port(), "127.0.0.1");
    const std::vector<uint8_t> packet = make_packet(0x1234U, 3U, 0x89abcdefU);
    sender.send_to(packet, input.bound_port());

    ltr2::Block block{};
    double kernel_wall = 0.0;
    require(input.receive(block, kernel_wall), "valid datagram not received");
    require(block.frame == 0x1234U && block.index == 3U && block.stamp == 0x89abcdefU,
            "decoded block header mismatch");
    require(block.ranges.front() == 1U && block.ranges.back() == 256U,
            "decoded ranges mismatch");
    require(block.intensities.front() == 2000U && block.intensities.back() == 2255U,
            "decoded intensities mismatch");
    const double current_wall = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    require(kernel_wall > 0.0 && kernel_wall <= current_wall + 1.0 &&
                current_wall - kernel_wall < 5.0,
            "kernel timestamp is not a current wall timestamp");
    require(!input.receive(block, kernel_wall), "empty socket reported a packet");
}

void test_wrong_source_rejected() {
    Sender expected("127.0.0.1");
    Sender wrong_port("127.0.0.1");
    Sender wrong_ip("127.0.0.2", expected.port());
    ltr2::UdpInput input("127.0.0.1", 0U, expected.port(), "127.0.0.1");
    const std::vector<uint8_t> packet = make_packet(9U, 0U, 1000U);
    ltr2::Block block{};
    double kernel_wall = 0.0;

    wrong_port.send_to(packet, input.bound_port());
    require(!input.receive(block, kernel_wall), "wrong source port accepted");
    wrong_ip.send_to(packet, input.bound_port());
    require(!input.receive(block, kernel_wall), "wrong source IP accepted");
    require(input.discarded_packets() == 2U, "wrong peer counter mismatch");

    expected.send_to(packet, input.bound_port());
    require(input.receive(block, kernel_wall), "receiver did not recover after wrong peers");
}

void test_malformed_packets_rejected() {
    Sender sender("127.0.0.1");
    ltr2::UdpInput input("127.0.0.1", 0U, sender.port(), "127.0.0.1");
    const std::vector<uint8_t> valid = make_packet(10U, 0U, 2000U);
    ltr2::Block block{};
    double kernel_wall = 0.0;

    std::vector<uint8_t> corrupted = valid;
    corrupted[200U] ^= 0x01U;
    sender.send_to(corrupted, input.bound_port());
    require(!input.receive(block, kernel_wall), "CRC-corrupted datagram accepted");

    const std::vector<uint8_t> short_packet(valid.begin(), valid.end() - 1);
    sender.send_to(short_packet, input.bound_port());
    require(!input.receive(block, kernel_wall), "short datagram accepted");

    std::vector<uint8_t> oversized = valid;
    oversized.push_back(0U);
    sender.send_to(oversized, input.bound_port());
    require(!input.receive(block, kernel_wall), "oversized datagram accepted");
    require(input.malformed_packets() == 3U, "malformed packet counter mismatch");

    sender.send_to(valid, input.bound_port());
    require(input.receive(block, kernel_wall), "receiver did not recover after malformed packets");
}

void test_exclusive_bind_cleanup_and_drain() {
    Sender sender("127.0.0.1");
    uint16_t receiver_port = 0U;
    {
        ltr2::UdpInput input("127.0.0.1", 0U, sender.port(), "127.0.0.1");
        receiver_port = input.bound_port();
        require_runtime_error(
            [&]() {
                ltr2::UdpInput duplicate(
                    "127.0.0.1", receiver_port, sender.port(), "127.0.0.1");
            },
            "second receiver acquired an exclusive port");

        const std::vector<uint8_t> packet = make_packet(11U, 0U, 3000U);
        for (unsigned index = 0U; index < 8U; ++index) {
            sender.send_to(packet, receiver_port);
        }
        input.drain();
        ltr2::Block block{};
        double kernel_wall = 0.0;
        require(!input.receive(block, kernel_wall), "drain left queued datagrams");
    }

    ltr2::UdpInput rebound("127.0.0.1", receiver_port, sender.port(), "127.0.0.1");
    require(rebound.bound_port() == receiver_port, "cleanup did not release receiver port");
}

}

int main() {
    try {
        test_valid_packet_and_timestamp();
        test_wrong_source_rejected();
        test_malformed_packets_rejected();
        test_exclusive_bind_cleanup_and_drain();
        std::cout << "ltr2 UDP input tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "ltr2 UDP input test failed: " << error.what() << '\n';
        return 1;
    }
}
