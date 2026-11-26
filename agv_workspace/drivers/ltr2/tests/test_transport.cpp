#include "ltr2/ethernet.hpp"
#include <iostream>
#include <functional>

static void require(bool condition) {
    if (!condition) {
        throw std::runtime_error("transport test failed");
    }
}

static void checksum(std::vector<uint8_t>& packet) {
    packet[24] = packet[25] = 0;
    uint32_t sum = 0;
    for (size_t offset = 14; offset < 34; offset += 2) {
        sum += ltr2::network_u16(packet.data() + offset);
    }
    while (sum >> 16) {
        sum = (sum & 65535) + (sum >> 16);
    }
    const uint16_t value = static_cast<uint16_t>(~sum);
    packet[24] = value >> 8;
    packet[25] = value & 255;
}

static bool accepts(const std::vector<uint8_t>& packet) {
    const uint8_t* payload = nullptr;
    size_t length = 0;
    try {
        return ltr2::ethernet_payload(packet, payload, length);
    } catch (const std::exception&) {
        return false;
    }
}

int main() {
    try {
        std::vector<uint8_t> packet(1082);
        packet[12] = 8;
        packet[14] = 0x45;
        packet[16] = 4;
        packet[17] = 44;
        packet[22] = 64;
        packet[23] = 17;
        packet[26] = 192;
        packet[27] = 168;
        packet[28] = 10;
        packet[29] = 160;
        packet[30] = packet[31] = packet[32] = packet[33] = 255;
        packet[34] = packet[36] = 8;
        packet[35] = packet[37] = 57;
        packet[38] = 4;
        packet[39] = 24;
        checksum(packet);
        require(accepts(packet));
        const uint8_t* payload = nullptr;
        size_t length = 0;
        require(ltr2::ethernet_payload(packet, payload, length));
        require(length == 1040 && payload == packet.data() + 42);
        for (size_t size = 0; size < packet.size(); ++size) {
            require(!accepts(std::vector<uint8_t>(packet.begin(), packet.begin() + size)));
        }
        for (const size_t offset : {14, 16, 17, 20, 21, 24, 26, 29, 34, 35, 36, 37, 38, 39}) {
            auto corrupted = packet;
            corrupted[offset] ^= 1;
            require(!accepts(corrupted));
        }
        for (const uint8_t flags : {0x20, 0x80}) {
            auto fragmented = packet;
            fragmented[20] = flags;
            checksum(fragmented);
            require(!accepts(fragmented));
        }
        auto padded = packet;
        padded.resize(1100);
        require(accepts(padded));
        std::cout << "PASS transport bounds, source, ports, IPv4 checksum, fragments, padding\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
