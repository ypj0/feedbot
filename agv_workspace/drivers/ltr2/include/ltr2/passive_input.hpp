#pragma once

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <linux/if_packet.h>
#include <net/if.h>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

namespace ltr2 {

inline double monotonic_seconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

inline double wall_seconds() {
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

class PassiveInput {
public:
    explicit PassiveInput(const std::string& interface) {
        const unsigned interface_index = if_nametoindex(interface.c_str());
        if (!interface_index) {
            throw std::runtime_error("unknown interface");
        }
        descriptor_ = socket(AF_PACKET, SOCK_RAW, htons(0x0800));
        if (descriptor_ < 0) {
            throw std::runtime_error("AF_PACKET requires root or CAP_NET_RAW");
        }
        sockaddr_ll address{};
        address.sll_family = AF_PACKET;
        address.sll_protocol = htons(0x0800);
        address.sll_ifindex = static_cast<int>(interface_index);
        int enabled = 1;
        timeval timeout{0, 50000};
        if (bind(descriptor_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) ||
            setsockopt(descriptor_, SOL_SOCKET, SO_TIMESTAMPNS, &enabled, sizeof(enabled)) ||
            setsockopt(descriptor_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout))) {
            close(descriptor_);
            descriptor_ = -1;
            throw std::runtime_error("passive socket setup failed");
        }
    }

    ~PassiveInput() {
        if (descriptor_ >= 0) {
            close(descriptor_);
        }
    }

    PassiveInput(const PassiveInput&) = delete;
    PassiveInput& operator=(const PassiveInput&) = delete;

    bool receive(std::vector<uint8_t>& packet, double& stamp) {
        packet.resize(65535);
        alignas(cmsghdr) char ancillary[CMSG_SPACE(sizeof(timespec))]{};
        sockaddr_ll sender{};
        iovec buffer{packet.data(), packet.size()};
        msghdr message{};
        message.msg_name = &sender;
        message.msg_namelen = sizeof(sender);
        message.msg_iov = &buffer;
        message.msg_iovlen = 1;
        message.msg_control = ancillary;
        message.msg_controllen = sizeof(ancillary);
        const ssize_t received = recvmsg(descriptor_, &message, 0);
        if (received < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                return false;
            }
            throw std::runtime_error("passive receive failed");
        }
        if (sender.sll_pkttype == PACKET_OUTGOING) {
            return false;
        }
        if (message.msg_flags & (MSG_TRUNC | MSG_CTRUNC)) {
            ++truncated_receives_;
            return false;
        }
        bool found = false;
        for (cmsghdr* control = CMSG_FIRSTHDR(&message); control;
             control = CMSG_NXTHDR(&message, control)) {
            if (control->cmsg_level == SOL_SOCKET && control->cmsg_type == SO_TIMESTAMPNS &&
                control->cmsg_len >= CMSG_LEN(sizeof(timespec))) {
                timespec timestamp{};
                std::memcpy(&timestamp, CMSG_DATA(control), sizeof(timestamp));
                if (timestamp.tv_nsec < 0 || timestamp.tv_nsec >= 1000000000) {
                    throw std::runtime_error("invalid kernel timestamp");
                }
                stamp = timestamp.tv_sec + timestamp.tv_nsec * 1e-9;
                found = true;
            }
        }
        if (!found || !std::isfinite(stamp)) {
            throw std::runtime_error("missing kernel timestamp");
        }
        packet.resize(static_cast<size_t>(received));
        return true;
    }

    uint64_t truncated_receives() const {
        return truncated_receives_;
    }

private:
    int descriptor_ = -1;
    uint64_t truncated_receives_ = 0;
};

}
