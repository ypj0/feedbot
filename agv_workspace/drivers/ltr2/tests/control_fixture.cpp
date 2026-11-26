#include "ltr2/lifecycle.hpp"
#include "ltr2/sdk_control.hpp"

#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char** argv) {
    try {
        if (argc != 2) {
            throw std::runtime_error("usage: ltr2_control_fixture PORT");
        }
        const int parsed_port = std::stoi(argv[1]);
        if (parsed_port <= 0 || parsed_port > 65535) {
            throw std::runtime_error("invalid fixture port");
        }
        double now = 1.0;
        ltr2::SdkControl control("127.0.0.1", "127.0.0.1",
                                 static_cast<uint16_t>(parsed_port), 2105, "LT-R2-B");
        ltr2::Lifecycle lifecycle(control, [&now]() { return now; });
        lifecycle.claim();
        if (!lifecycle.configure() || !lifecycle.activate() || lifecycle.ready()) {
            throw std::runtime_error("initial configure or activation failed");
        }
        now = 1.1;
        if (!lifecycle.frame(lifecycle.epoch(), now, now) || !lifecycle.stop(true)) {
            throw std::runtime_error("initial readiness or sleep failed");
        }
        now = 2.0;
        if (!lifecycle.activate() || lifecycle.ready()) {
            throw std::runtime_error("explicit wake failed");
        }
        now = 2.1;
        if (!lifecycle.frame(lifecycle.epoch(), now, now) || !lifecycle.stop(false) ||
            !lifecycle.release()) {
            throw std::runtime_error("wake readiness, stop, or release failed");
        }
        std::cout << "PASS real LDCP wrapper against loopback fixture only\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 2;
    }
}
