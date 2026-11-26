#pragma once

#include "ltr2/core.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

namespace ltr2 {

class UdpInput {
public:
    explicit UdpInput(std::string source_ip, uint16_t local_port = 2105U,
                      uint16_t source_port = 2105U, std::string bind_ip = "0.0.0.0")
        : source_port_(source_port) {
        if (source_port_ == 0U ||
            inet_pton(AF_INET, source_ip.c_str(), &source_address_) != 1) {
            throw std::runtime_error("invalid UDP source endpoint");
        }

        sockaddr_in local{};
        local.sin_family = AF_INET;
        local.sin_port = htons(local_port);
        if (inet_pton(AF_INET, bind_ip.c_str(), &local.sin_addr) != 1) {
            throw std::runtime_error("invalid UDP bind address");
        }

        descriptor_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (descriptor_ < 0) {
            throw_socket_error("UDP socket");
        }

        if (bind(descriptor_, reinterpret_cast<const sockaddr*>(&local), sizeof(local)) != 0) {
            const int error = errno;
            close_descriptor();
            throw_socket_error("exclusive UDP bind", error);
        }

        int enabled = 1;
        if (setsockopt(descriptor_, SOL_SOCKET, SO_TIMESTAMPNS, &enabled, sizeof(enabled)) != 0) {
            const int error = errno;
            close_descriptor();
            throw_socket_error("UDP timestamp setup", error);
        }

        socklen_t local_size = sizeof(local);
        if (getsockname(descriptor_, reinterpret_cast<sockaddr*>(&local), &local_size) != 0 ||
            local_size != sizeof(local) || local.sin_family != AF_INET) {
            const int error = errno;
            close_descriptor();
            throw_socket_error("UDP bound endpoint", error);
        }
        bound_port_ = ntohs(local.sin_port);
    }

    ~UdpInput() {
        close_descriptor();
    }

    UdpInput(const UdpInput&) = delete;
    UdpInput& operator=(const UdpInput&) = delete;

    bool receive(Block& block, double& kernel_wall) {
        std::array<uint8_t, packet_size_> bytes{};
        alignas(cmsghdr) std::array<char, CMSG_SPACE(sizeof(timespec))> ancillary{};
        sockaddr_in peer{};
        iovec vector{};
        vector.iov_base = bytes.data();
        vector.iov_len = bytes.size();
        msghdr message{};
        message.msg_name = &peer;
        message.msg_namelen = sizeof(peer);
        message.msg_iov = &vector;
        message.msg_iovlen = 1U;
        message.msg_control = ancillary.data();
        message.msg_controllen = ancillary.size();

        const ssize_t received = recvmsg(descriptor_, &message, 0);
        if (received < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                return false;
            }
            throw_socket_error("UDP receive");
        }

        if (message.msg_namelen != sizeof(peer) || peer.sin_family != AF_INET ||
            peer.sin_addr.s_addr != source_address_.s_addr ||
            ntohs(peer.sin_port) != source_port_) {
            ++discarded_packets_;
            return false;
        }
        if ((message.msg_flags & (MSG_TRUNC | MSG_CTRUNC)) != 0 ||
            received != static_cast<ssize_t>(bytes.size())) {
            ++malformed_packets_;
            return false;
        }

        timespec timestamp{};
        bool have_timestamp = false;
        for (cmsghdr* control = CMSG_FIRSTHDR(&message); control != nullptr;
             control = CMSG_NXTHDR(&message, control)) {
            if (control->cmsg_level == SOL_SOCKET && control->cmsg_type == SO_TIMESTAMPNS &&
                control->cmsg_len >= CMSG_LEN(sizeof(timestamp))) {
                std::memcpy(&timestamp, CMSG_DATA(control), sizeof(timestamp));
                have_timestamp = true;
                break;
            }
        }
        if (!have_timestamp) {
            throw std::runtime_error("UDP kernel timestamp missing");
        }

        Block decoded{};
        try {
            decoded = decode(bytes.data(), bytes.size());
        } catch (const std::runtime_error&) {
            ++malformed_packets_;
            return false;
        }
        block = decoded;
        kernel_wall = static_cast<double>(timestamp.tv_sec) +
                      static_cast<double>(timestamp.tv_nsec) * 1.0e-9;
        return true;
    }

    void drain() {
        std::array<uint8_t, packet_size_> bytes{};
        for (std::size_t attempt = 0U; attempt < drain_limit_; ++attempt) {
            const ssize_t received = recv(descriptor_, bytes.data(), bytes.size(), 0);
            if (received >= 0) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return;
            }
            if (errno == EINTR) {
                continue;
            }
            throw_socket_error("UDP drain");
        }

        const ssize_t queued = recv(descriptor_, bytes.data(), bytes.size(), MSG_PEEK);
        if (queued < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            return;
        }
        if (queued < 0) {
            throw_socket_error("UDP drain check");
        }
        throw std::runtime_error("UDP drain limit exceeded");
    }

    uint16_t bound_port() const noexcept {
        return bound_port_;
    }

    uint64_t discarded_packets() const noexcept {
        return discarded_packets_;
    }

    uint64_t malformed_packets() const noexcept {
        return malformed_packets_;
    }

private:
    [[noreturn]] static void throw_socket_error(const char* operation, int error = errno) {
        throw std::runtime_error(std::string(operation) + ": " + std::strerror(error));
    }

    void close_descriptor() noexcept {
        if (descriptor_ >= 0) {
            close(descriptor_);
            descriptor_ = -1;
        }
    }

    static constexpr std::size_t packet_size_ = 1040U;
    static constexpr std::size_t drain_limit_ = 512U;
    int descriptor_ = -1;
    in_addr source_address_{};
    uint16_t source_port_ = 0U;
    uint16_t bound_port_ = 0U;
    uint64_t discarded_packets_ = 0U;
    uint64_t malformed_packets_ = 0U;
};

}
