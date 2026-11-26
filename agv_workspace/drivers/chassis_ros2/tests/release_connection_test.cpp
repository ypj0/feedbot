#include "feedbot_chassis_ros2/release_connection.hpp"
#include <cassert>
#include <future>
#include <iostream>
#ifdef __APPLE__
#include <util.h>
#else
#include <pty.h>
#endif

using namespace feedbot_chassis_ros2;

struct Connection {
    int master = -1;
    int descriptor = -1;
    std::string path;
    std::unique_ptr<feedbot_chassis::SerialPort> port;
    std::unique_ptr<Session> session;

    Connection() {
        int slave;
        char name[128];
        assert(openpty(&master, &slave, name, nullptr, nullptr) == 0);
        close(slave);
        path = name;
        port = std::make_unique<feedbot_chassis::SerialPort>(path, true);
        descriptor = port->fd();
        session = std::make_unique<Session>(descriptor);
    }

    ~Connection() { if (master >= 0) close(master); }

    void assert_closed() const {
        assert(!port && !session);
        assert(fcntl(descriptor, F_GETFD) == -1 && errno == EBADF);
    }
};

void normal_release() {
    Connection connection;
    auto released = release_connection(connection.session, connection.port);
    connection.assert_closed();
    assert(!released.stop_error && released.snapshot);
    assert(released.snapshot->tx_history().back().stopping);
    const auto stop_write = released.snapshot->tx_history().back();
    assert(stop_write.returned == 40);
    assert(stop_write.frame == feedbot_chassis::kReviewedStationary);
    auto again = release_connection(connection.session, connection.port);
    assert(!again.snapshot && !again.stop_error);
    feedbot_chassis::SerialPort reopened(connection.path, true);
}

void stop_failure_still_releases() {
    Connection connection;
    close(connection.master);
    connection.master = -1;
    auto released = release_connection(connection.session, connection.port);
    connection.assert_closed();
    assert(released.stop_error && released.snapshot);
    bool explicit_error = false;
    try { std::rethrow_exception(released.stop_error); }
    catch (const std::exception &error) {
        explicit_error = std::string(error.what()) == "serial write failed";
    }
    assert(explicit_error);
    assert(released.snapshot->tx_history().back().returned == -1);
    assert(released.snapshot->tx_history().back().stopping);
}

void throwing_diagnostics_after_release() {
    Connection connection;
    auto released = release_connection(connection.session, connection.port);
    const auto publish = [&] {
        connection.assert_closed();
        assert(!released.snapshot->tx_history_json().empty());
        throw std::runtime_error("injected diagnostic publication failure");
    };
    bool failed = false;
    try { publish(); }
    catch (const std::runtime_error &) { failed = true; }
    assert(failed);
    connection.assert_closed();
}

void blocked_diagnostics_after_release() {
    Connection connection;
    std::promise<void> entered_diagnostics;
    auto entered = entered_diagnostics.get_future();
    std::promise<void> finish_diagnostics;
    auto finish = finish_diagnostics.get_future();
    auto worker = std::async(std::launch::async, [&] {
        auto released = release_connection(connection.session, connection.port);
        entered_diagnostics.set_value();
        finish.wait();
        assert(released.snapshot->tx_history().back().stopping);
    });
    assert(entered.wait_for(std::chrono::seconds(2)) == std::future_status::ready);
    connection.assert_closed();
    assert(worker.wait_for(std::chrono::milliseconds(10)) == std::future_status::timeout);
    {
        feedbot_chassis::SerialPort reopened(connection.path, true);
    }
    finish_diagnostics.set_value();
    worker.get();
}

void handshake_failure_ownership() {
    Connection connection;
    connection.session.reset();
    auto released = release_connection(connection.session, connection.port);
    connection.assert_closed();
    assert(!released.snapshot && !released.stop_error);
}

int main() {
    normal_release();
    stop_failure_still_releases();
    throwing_diagnostics_after_release();
    blocked_diagnostics_after_release();
    handshake_failure_ownership();
    std::cout << "PASS PTY release, stop encoding, retained evidence, exclusive reopen, stop failure, "
                 "diagnostic exception/block after release, no-session ownership, repeated cleanup\n";
}
