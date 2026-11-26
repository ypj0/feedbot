#pragma once
#include "session.hpp"
#include <exception>
#include <memory>

namespace feedbot_chassis_ros2 {
struct ReleasedConnection {
    std::unique_ptr<Session> snapshot;
    std::exception_ptr stop_error;
};

inline ReleasedConnection release_connection(std::unique_ptr<Session> &session,
                                             std::unique_ptr<feedbot_chassis::SerialPort> &port) {
    ReleasedConnection released{std::move(session), nullptr};
    if (released.snapshot) {
        try { released.snapshot->stop(); }
        catch (...) { released.stop_error = std::current_exception(); }
    }
    port.reset();
    return released;
}
}
