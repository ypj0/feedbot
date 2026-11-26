#include "feedbot_chassis_ros2/session.hpp"
#include <cassert>
#include <cstring>
#include <dlfcn.h>
#include <iostream>
#include <sys/socket.h>
#include <vector>

using feedbot_chassis_ros2::Frame;
using feedbot_chassis_ros2::Session;

struct WriteResult {
    ssize_t count;
    int error;
};

int intercepted_descriptor = -1;
Frame expected_frame{};
size_t accepted_bytes = 0;
size_t write_calls = 0;
std::vector<WriteResult> write_results;

extern "C" ssize_t write(int descriptor, const void *buffer, size_t length) {
    using Writer = ssize_t (*)(int, const void *, size_t);
    static const auto original_write = reinterpret_cast<Writer>(dlsym(RTLD_NEXT, "write"));
    assert(original_write != nullptr);
    if (descriptor != intercepted_descriptor) return original_write(descriptor, buffer, length);
    assert(write_calls < write_results.size());
    assert(length == expected_frame.size() - accepted_bytes);
    assert(std::memcmp(buffer, expected_frame.data() + accepted_bytes, length) == 0);
    const auto result = write_results[write_calls++];
    if (result.count > 0) {
        assert(static_cast<size_t>(result.count) <= length);
        const auto actual = original_write(descriptor, buffer, static_cast<size_t>(result.count));
        assert(actual == result.count);
        accepted_bytes += static_cast<size_t>(actual);
    }
    errno = result.error;
    return result.count;
}

struct SocketPair {
    int descriptors[2];
    SocketPair() {
        assert(socketpair(AF_UNIX, SOCK_STREAM, 0, descriptors) == 0);
        for (const auto descriptor : descriptors) {
            assert(fcntl(descriptor, F_SETFL, O_NONBLOCK) == 0);
        }
    }
    ~SocketPair() {
        intercepted_descriptor = -1;
        for (const auto descriptor : descriptors) close(descriptor);
    }
};

void script(int descriptor, const Frame &frame, std::vector<WriteResult> results) {
    intercepted_descriptor = descriptor;
    expected_frame = frame;
    accepted_bytes = 0;
    write_calls = 0;
    write_results = std::move(results);
}

template<class Function> std::string failure(Function function) {
    try { function(); }
    catch (const std::exception &error) { return error.what(); }
    assert(false);
    return "";
}

bool firmware_checksum_accepts(const Frame &frame) {
    uint8_t sum = 0;
    for (size_t index = 0; index < frame.size() - 3; ++index) sum += frame[index];
    return frame[0] == 0x7f && frame[1] == 0x7f && frame[38] == 0x0d &&
           frame[39] == 0x0a && frame[37] == static_cast<uint8_t>(0 - sum);
}

void successful_writes(const std::vector<WriteResult> &results) {
    SocketPair sockets;
    Session session(sockets.descriptors[0]);
    script(sockets.descriptors[0], feedbot_chassis::kReviewedStationary, results);
    session.command(0, 0);
    assert(write_calls == results.size() && accepted_bytes == 40);
    assert(session.tx_attempt_count() == results.size());
    const auto history = session.tx_history();
    assert(history.size() == results.size());
    size_t offset = 0;
    for (size_t index = 0; index < history.size(); ++index) {
        const auto &attempt = history[index];
        assert(attempt.sequence == index + 1 && attempt.frame_sequence == 1);
        assert(attempt.frame == expected_frame && attempt.offset == offset);
        assert(attempt.returned == results[index].count);
        assert(attempt.error == (results[index].count < 0 ? results[index].error : 0));
        assert(!attempt.stopping && attempt.finished >= attempt.started);
        if (index > 0) assert(attempt.started >= history[index - 1].finished);
        if (attempt.returned > 0) offset += static_cast<size_t>(attempt.returned);
    }
    if (results.size() == 5) std::cout << "TX_HISTORY_JSON=" << session.tx_history_json() << "\n";
    Frame received{};
    assert(::read(sockets.descriptors[1], received.data(), received.size()) == 40);
    assert(received == expected_frame && firmware_checksum_accepts(received));
    assert(session.command_zero_count == 1 && session.command_nonzero_count == 0);
    uint8_t extra;
    assert(::read(sockets.descriptors[1], &extra, 1) == -1);
    assert(errno == EAGAIN || errno == EWOULDBLOCK);
}

void failed_partial_write(WriteResult terminal) {
    SocketPair sockets;
    Session session(sockets.descriptors[0]);
    script(sockets.descriptors[0], feedbot_chassis::kReviewedStationary, {{9, 0}, terminal});
    assert(failure([&] { session.command(0, 0); }) == "serial write failed");
    assert(accepted_bytes == 9 && write_calls == 2);
    assert(session.command_zero_count == 0);
    const auto history = session.tx_history();
    assert(history.size() == 2 && history[1].offset == 9);
    assert(history[1].returned == terminal.count);
    assert(history[1].error == (terminal.count < 0 ? terminal.error : 0));
    failure([&] { session.command(0, 0); });
    failure([&] { session.stop(); });
    assert(write_calls == 2);
    Frame received{};
    assert(::read(sockets.descriptors[1], received.data(), received.size()) == 9);
    assert(std::memcmp(received.data(), expected_frame.data(), 9) == 0);
    assert(::read(sockets.descriptors[1], received.data(), received.size()) == -1);
    assert(errno == EAGAIN || errno == EWOULDBLOCK);
}

void recorded_negative_ack() {
    const std::string hex = "7f7f2802800000000000000000000000000000000000000000000000000000000000000000580d0a";
    Frame reply{};
    for (size_t index = 0; index < reply.size(); ++index) {
        reply[index] = static_cast<uint8_t>(std::stoul(hex.substr(index * 2, 2), nullptr, 16));
    }
    assert(firmware_checksum_accepts(reply));
    assert(feedbot_chassis::valid(reply));
    SocketPair sockets;
    Session session(sockets.descriptors[0]);
    assert(::write(sockets.descriptors[1], reply.data(), reply.size()) == 40);
    assert(failure([&] { session.receive(); }) == "invalid runtime feedback; frame_hex=" + hex);
    assert(session.runtime_field_error_count == 1 && session.feedback_count == 0);
    assert(session.rejected() == 0 && session.feedback_timeout_count == 0);
    assert(session.wheels.left == 0 && session.wheels.right == 0);
}

void history_wraps() {
    SocketPair sockets;
    Session session(sockets.descriptors[0]);
    for (size_t index = 0; index < 140; ++index) {
        script(sockets.descriptors[0], feedbot_chassis::kReviewedStationary, {{40, 0}});
        session.command(0, 0);
        Frame received{};
        assert(::read(sockets.descriptors[1], received.data(), received.size()) == 40);
    }
    auto history = session.tx_history();
    assert(history.size() == 128 && session.tx_attempt_count() == 140);
    assert(history.front().sequence == 13 && history.back().sequence == 140);
    for (size_t index = 1; index < history.size(); ++index) {
        assert(history[index].sequence == history[index - 1].sequence + 1);
        assert(history[index].frame_sequence == history[index].sequence);
    }
    script(sockets.descriptors[0], feedbot_chassis::kReviewedStationary, {{40, 0}});
    session.stop();
    history = session.tx_history();
    assert(history.size() == 128 && history.front().sequence == 14);
    assert(history.back().sequence == 141 && history.back().stopping);
    assert(history[126].stopping == false);
}

int main() {
    for (int linear_mm = -500; linear_mm <= 500; linear_mm += 25) {
        for (int angular_mrad = -1000; angular_mrad <= 1000; angular_mrad += 25) {
            const auto frame = feedbot_chassis::velocity_frame({}, linear_mm / 1000.0,
                                                              angular_mrad / 1000.0);
            assert(firmware_checksum_accepts(frame));
            auto corrupted = frame;
            corrupted[10] ^= 1;
            assert(!firmware_checksum_accepts(corrupted));
        }
    }
    successful_writes({{40, 0}});
    successful_writes({{13, 0}, {27, 0}});
    successful_writes({{13, 0}, {-1, EINTR}, {-1, EAGAIN}, {5, EINTR}, {22, 0}});
    failed_partial_write({-1, EIO});
    failed_partial_write({0, 0});
    recorded_negative_ack();
    history_wraps();
    std::cout << "PASS 3321 checksummed commands, corrupt-request rejection, exact short-write suffix, "
                 "EINTR/EAGAIN offsets, partial failure without frame splice, recorded ACK=0x80 rejection, "
                 "TX history fields/chronology/wrap/stop provenance\n";
}
