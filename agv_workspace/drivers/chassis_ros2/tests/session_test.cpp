#include "feedbot_chassis_ros2/session.hpp"
#include <cassert>
#include <iostream>
#include <limits>
#include <pty.h>
#include <sys/socket.h>

using namespace feedbot_chassis_ros2;

Frame feedback_frame(int left, int right) {
    Frame frame = feedbot_chassis::kReviewedStationary;
    frame[4] = 0x60;
    frame[5] = 0;
    frame[6] = 1;
    frame[8] = 0;
    frame[9] = frame[10] = 0;
    frame[28] = 0;
    feedbot_chassis::be16(frame, 11, left);
    feedbot_chassis::be16(frame, 13, right);
    feedbot_chassis::be16(frame, 35, 441);
    feedbot_chassis::checksum(frame);
    return frame;
}

template<class Function> void fails(Function function) {
    bool failed = false;
    try { function(); } catch (const std::exception &) { failed = true; }
    assert(failed);
}

void near(double actual, double expected) { assert(std::abs(actual - expected) < 1e-10); }

std::string frame_hex(const Frame &frame) {
    std::ostringstream output;
    output << std::hex << std::setfill('0');
    for (const auto byte : frame) output << std::setw(2) << unsigned(byte);
    return output.str();
}

void timing_recorded(const Session &session) {
    assert(session.first_read_return_monotonic_s > 0.0);
    assert(session.receive_total_last_us >= 0.0);
    assert(session.receive_total_max_us >= session.receive_total_last_us);
    assert(session.read_syscall_total_last_us >= 0.0);
    assert(session.read_syscall_total_max_us >= session.read_syscall_total_last_us);
    assert(session.parse_validate_total_last_us >= 0.0);
    assert(session.parse_validate_total_max_us >= session.parse_validate_total_last_us);
}

struct SocketPair {
    int descriptors[2];
    SocketPair() {
        assert(socketpair(AF_UNIX, SOCK_STREAM, 0, descriptors) == 0);
        for (int descriptor : descriptors) assert(fcntl(descriptor, F_SETFL, O_NONBLOCK) == 0);
    }
    ~SocketPair() { for (int descriptor : descriptors) close(descriptor); }
};

int main() {
    Wheels wheels;
    for (int index = 0; index < 100; ++index) wheels.accumulate(feedbot_chassis::feedback(feedback_frame(100, 100)));
    near(wheels.left, 0.1 / Wheels::radius);
    near(wheels.right, wheels.left);
    wheels.accumulate(feedbot_chassis::feedback(feedback_frame(-100, 0)));
    near(wheels.left, 0.099 / Wheels::radius);
    near(wheels.right, 0.1 / Wheels::radius);
    assert(Wheels::command(0, 0) == feedbot_chassis::kReviewedStationary);
    const auto straight = Wheels::command(1, 1);
    assert(feedbot_chassis::signed_be(straight, 10) == 85);
    assert(feedbot_chassis::signed_be(straight, 12) == 0);
    const auto spin = Wheels::command(-1, 1);
    assert(feedbot_chassis::signed_be(spin, 10) == 0);
    assert(feedbot_chassis::signed_be(spin, 12) == 374);
    for (size_t index = 0; index < 40; ++index) {
        if (index != 10 && index != 11 && index != 12 && index != 13 && index != 37)
            assert(spin[index] == feedbot_chassis::kReviewedStationary[index]);
    }
    fails([] { Wheels::command(std::numeric_limits<double>::quiet_NaN(), 0); });
    fails([] { Wheels::command(100, 100); });
    {
        SocketPair pair;
        Session session(pair.descriptors[0]);
        fails([&] { session.command(1, 1); });
        auto frame = feedback_frame(100, 200);
        assert(::write(pair.descriptors[1], frame.data(), 17) == 17);
        session.receive();
        assert(session.feedback_count == 0);
        timing_recorded(session);
        assert(::write(pair.descriptors[1], frame.data() + 17, 23) == 23);
        for (int index = 0; index < 80; ++index) assert(::write(pair.descriptors[1], frame.data(), 40) == 40);
        session.receive();
        assert(session.feedback_count == 81);
        assert(session.first_feedback_monotonic_s > 0.0);
        assert(session.observed_frame_hex() == frame_hex(frame));
        assert(session.last_observed_frame_type == 2 && session.last_observed_frame_ack == 0x60);
        assert(session.last_observed_frame_monotonic_s >= session.first_feedback_monotonic_s);
        timing_recorded(session);
        near(session.wheels.left, 81 * 0.001 / Wheels::radius);
        near(session.wheels.right, 81 * 0.002 / Wheels::radius);
        session.command(1, 1);
        Frame transmitted;
        assert(::read(pair.descriptors[1], transmitted.data(), 40) == 40);
        assert(transmitted == straight);
        assert(session.command_nonzero_count == 1 && session.command_zero_count == 0);
        session.command(0, 0);
        assert(::read(pair.descriptors[1], transmitted.data(), 40) == 40);
        assert(transmitted == feedbot_chassis::kReviewedStationary);
        assert(session.command_nonzero_count == 1 && session.command_zero_count == 1);
        frame[8] = 1;
        frame[9] = 7;
        feedbot_chassis::checksum(frame);
        assert(::write(pair.descriptors[1], frame.data(), 40) == 40);
        session.receive();
        assert(session.status.charge == 1 && session.status.left_alarm == 7);
        fails([&] { session.command(1, 1); });
        session.stop();
        assert(::read(pair.descriptors[1], transmitted.data(), 40) == 40);
        assert(transmitted == feedbot_chassis::kReviewedStationary);
        fails([&] { session.command(1, 1); });
        fails([&] { session.receive(); });
    }
    {
        SocketPair pair;
        Session session(pair.descriptors[0]);
        auto frame = feedback_frame(1, 1);
        frame[37] ^= 1;
        assert(::write(pair.descriptors[1], frame.data(), 40) == 40);
        frame = feedback_frame(2, 3);
        assert(::write(pair.descriptors[1], frame.data(), 40) == 40);
        session.receive();
        assert(session.feedback_count == 1 && session.rejected() > 0);
        const auto observed_hex = session.observed_frame_hex();
        const auto observed_monotonic = session.last_observed_frame_monotonic_s;
        assert(session.last_rejection_monotonic > 0.0);
        assert(session.feedback_timeout_count == 0 && session.runtime_field_error_count == 0);
        near(session.wheels.left, 0.00002 / Wheels::radius);
        std::this_thread::sleep_for(std::chrono::milliseconds(210));
        fails([&] { session.receive(); });
        fails([&] { session.command(1, 1); });
        assert(session.feedback_timeout_count == 1 && session.runtime_field_error_count == 0);
        assert(session.last_receive_fault_monotonic >= session.last_rejection_monotonic);
        assert(session.observed_frame_hex() == observed_hex);
        assert(session.last_observed_frame_monotonic_s == observed_monotonic);
        timing_recorded(session);
        near(session.wheels.left, 0.00002 / Wheels::radius);
    }
    {
        SocketPair pair;
        Session session(pair.descriptors[0]);
        auto valid = feedback_frame(4, 5);
        assert(::write(pair.descriptors[1], valid.data(), 40) == 40);
        session.receive();
        const auto observed_hex = session.observed_frame_hex();
        const auto observed_monotonic = session.last_observed_frame_monotonic_s;
        auto corrupt = feedback_frame(6, 7);
        corrupt[37] ^= 1;
        assert(::write(pair.descriptors[1], corrupt.data(), 40) == 40);
        session.receive();
        assert(session.rejected() > 0);
        assert(session.observed_frame_hex() == observed_hex);
        assert(session.last_observed_frame_monotonic_s == observed_monotonic);
    }
    {
        SocketPair pair;
        Session session(pair.descriptors[0]);
        auto frame = feedback_frame(100, 100);
        feedbot_chassis::be16(frame, 35, 437);
        feedbot_chassis::checksum(frame);
        assert(::write(pair.descriptors[1], frame.data(), 40) == 40);
        fails([&] { session.receive(); });
        assert(session.feedback_count == 0);
        near(session.wheels.left, 0);
    }
    for (const auto &header : {std::pair<int, int>{1, 0x60}, {3, 0x60}, {2, 0}}) {
        SocketPair pair;
        Session session(pair.descriptors[0]);
        auto frame = feedback_frame(100, 100);
        frame[3] = header.first;
        frame[4] = header.second;
        feedbot_chassis::checksum(frame);
        assert(feedbot_chassis::valid(frame));
        assert(::write(pair.descriptors[1], frame.data(), frame.size()) == 40);
        std::string error_message;
        try { session.receive(); }
        catch (const std::runtime_error &error) { error_message = error.what(); }
        const std::string prefix = "invalid runtime feedback; frame_hex=";
        assert(error_message.rfind(prefix, 0) == 0);
        const auto hex = error_message.substr(prefix.size());
        assert(hex.size() == 80);
        for (size_t index = 0; index < frame.size(); ++index)
            assert(std::stoul(hex.substr(index * 2, 2), nullptr, 16) == frame[index]);
        assert(session.feedback_count == 0 && session.rejected() == 0);
        assert(session.runtime_field_error_count == 1 && session.feedback_timeout_count == 0);
        assert(session.last_receive_fault_monotonic > 0.0);
        assert(session.observed_frame_hex() == frame_hex(frame));
        assert(session.last_observed_frame_type == unsigned(header.first));
        assert(session.last_observed_frame_ack == unsigned(header.second));
        assert(session.last_observed_frame_monotonic_s > 0.0);
        assert(session.first_feedback_monotonic_s == 0.0);
        timing_recorded(session);
        near(session.wheels.left, 0);
        near(session.wheels.right, 0);
    }
    {
        SocketPair pair;
        Session session(pair.descriptors[0]);
        std::array<uint8_t, 4096> filler{};
        while (::write(pair.descriptors[0], filler.data(), filler.size()) > 0) {}
        assert(errno == EAGAIN || errno == EWOULDBLOCK);
        const auto started = feedbot_chassis::steady_seconds();
        fails([&] { session.command(0, 0); });
        assert(feedbot_chassis::steady_seconds() - started >= 0.04);
        while (::read(pair.descriptors[1], filler.data(), filler.size()) > 0) {}
        fails([&] { session.command(0, 0); });
        fails([&] { session.stop(); });
        assert(::read(pair.descriptors[1], filler.data(), filler.size()) == -1);
    }
    {
        int master, slave;
        char path[128];
        assert(openpty(&master, &slave, path, nullptr, nullptr) == 0);
        close(slave);
        {
            feedbot_chassis::SerialPort port(path, true);
            termios settings{};
            assert(tcgetattr(port.fd(), &settings) == 0);
            assert(cfgetispeed(&settings) == B115200 && cfgetospeed(&settings) == B115200);
            assert((settings.c_cflag & CSIZE) == CS8 && !(settings.c_cflag & (PARENB | CSTOPB)));
            int exclusive = 0;
            assert(ioctl(port.fd(), TIOCGEXCL, &exclusive) == 0 && exclusive == 1);
            std::thread simulator([master] {
                Frame request{};
                size_t received = 0;
                while (received < 40) {
                    const auto count = ::read(master, request.data() + received, 40 - received);
                    assert(count > 0);
                    received += static_cast<size_t>(count);
                }
                assert(feedbot_chassis::valid(request) && request[3] == 1);
                Frame response = request;
                response[4] = 0x60;
                response[5] = 0;
                response[6] = 1;
                response[7] = 0xc2;
                response[8] = 0;
                feedbot_chassis::checksum(response);
                for (auto byte : response) assert(::write(master, &byte, 1) == 1);
            });
            port.handshake();
            simulator.join();
            Session session(port.fd());
            close(master);
            fails([&] { session.receive(); });
            timing_recorded(session);
        }
    }
    std::cout << "PASS incremental accumulation, no sign reversal, wheel inverse kinematics, all buffered frames, fragmentation, corruption, statuses, zero encoding, timing on success/fault, observed frame semantics, stale/no replay, blocked TX/no splice, handshake, 8N1, TIOCEXCL, PTY disconnect\n";
}
