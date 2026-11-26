#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace ltr2 {

inline uint16_t network_u16(const uint8_t* bytes) {
    return static_cast<uint16_t>((uint16_t(bytes[0]) << 8) | bytes[1]);
}

inline bool ethernet_payload(const std::vector<uint8_t>& packet,
                             const uint8_t*& payload, size_t& length) {
    payload = nullptr;
    length = 0;
    if (packet.size() < 34 || network_u16(packet.data() + 12) != 0x0800) {
        return false;
    }
    if (packet[26] != 192 || packet[27] != 168 || packet[28] != 10 || packet[29] != 160) {
        return false;
    }
    if ((packet[14] >> 4) != 4) {
        throw std::runtime_error("invalid IPv4 version");
    }
    if (packet[23] != 17) {
        return false;
    }
    const size_t header_length = (packet[14] & 15) * 4;
    const size_t total_length = network_u16(packet.data() + 16);
    if (header_length < 20 || total_length < header_length + 8 ||
        packet.size() < 14 + total_length ||
        (network_u16(packet.data() + 20) & 0xbfff) != 0) {
        throw std::runtime_error("invalid IPv4 length or fragmentation");
    }
    uint32_t checksum = 0;
    for (size_t offset = 14; offset < 14 + header_length; offset += 2) {
        checksum += network_u16(packet.data() + offset);
    }
    while (checksum >> 16) {
        checksum = (checksum & 0xffff) + (checksum >> 16);
    }
    if (checksum != 0xffff) {
        throw std::runtime_error("IPv4 checksum");
    }
    const size_t udp_offset = 14 + header_length;
    if (network_u16(packet.data() + udp_offset) != 2105 ||
        network_u16(packet.data() + udp_offset + 2) != 2105) {
        return false;
    }
    const size_t udp_length = network_u16(packet.data() + udp_offset + 4);
    if (udp_length != total_length - header_length) {
        throw std::runtime_error("UDP length");
    }
    payload = packet.data() + udp_offset + 8;
    length = udp_length - 8;
    return true;
}

}
