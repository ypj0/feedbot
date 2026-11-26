#include "ltr2/passive_input.hpp"
#include <iostream>

int main() {
    int sink = -1;
    int sender = -1;
    int result = 1;
    try {
        ltr2::PassiveInput input("lo");
        sink = socket(AF_INET, SOCK_DGRAM, 0);
        sender = socket(AF_INET, SOCK_DGRAM, 0);
        if (sink < 0 || sender < 0) {
            throw std::runtime_error("loopback socket creation");
        }
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (bind(sink, reinterpret_cast<sockaddr*>(&address), sizeof(address))) {
            throw std::runtime_error("ephemeral loopback bind");
        }
        socklen_t address_length = sizeof(address);
        if (getsockname(sink, reinterpret_cast<sockaddr*>(&address), &address_length)) {
            throw std::runtime_error("ephemeral loopback address");
        }
        std::vector<uint8_t> datagram(65507, 0);
        if (sendto(sender, datagram.data(), datagram.size(), 0,
                   reinterpret_cast<sockaddr*>(&address), sizeof(address)) !=
            static_cast<ssize_t>(datagram.size())) {
            throw std::runtime_error("loopback datagram send");
        }
        std::vector<uint8_t> packet;
        double stamp = 0;
        const double deadline = ltr2::monotonic_seconds() + 2.0;
        while (ltr2::monotonic_seconds() < deadline && input.truncated_receives() == 0) {
            input.receive(packet, stamp);
        }
        if (input.truncated_receives() == 0) {
            throw std::runtime_error("truncated receive was not exercised");
        }
        std::cout << "PASS loopback-only oversized datagram counted without receiver exit\n";
        result = 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
    }
    if (sink >= 0) {
        close(sink);
    }
    if (sender >= 0) {
        close(sender);
    }
    return result;
}
