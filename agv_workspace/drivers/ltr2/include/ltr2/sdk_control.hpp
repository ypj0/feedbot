#pragma once

#include "ltr2/control.hpp"

#include <arpa/inet.h>
#include <ldcp/device.h>
#include <ldcp/location.h>

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

namespace ltr2 {

class SdkControl final : public Control {
public:
    explicit SdkControl(const std::string& address, const std::string& target,
                        uint16_t control_port = 2105, uint16_t data_port = 2105,
                        const std::string& model = "LT-R2-B")
        : expected_model_(model), data_port_(htons(data_port)) {
        in_addr control_address{};
        in_addr target_address{};
        if (control_port == 0 || data_port == 0 || expected_model_.empty() ||
            inet_pton(AF_INET, address.c_str(), &control_address) != 1 ||
            inet_pton(AF_INET, target.c_str(), &target_address) != 1) {
            throw std::invalid_argument("valid IPv4 addresses, ports, and model required");
        }
        target_ = target_address.s_addr;
        ldcp_sdk::NetworkLocation location(control_address.s_addr, htons(control_port));
        device_.reset(new ldcp_sdk::Device(location));
        device_->setTimeout(500);
    }

    void open() override {
        check(device_->DeviceBase::open());
    }

    void validate() override {
        std::string model;
        bool oob_enabled = false;
        in_addr_t target = 0;
        in_port_t port = 0;
        check(device_->queryModel(model));
        check(device_->isOobEnabled(oob_enabled));
        check(device_->getOobTargetAddress(target));
        check(device_->getOobTargetPort(port));
        if (model != expected_model_ || !oob_enabled || target != target_ ||
            port != data_port_) {
            throw std::runtime_error(
                "device identity or OOB configuration mismatch: model=" + model +
                " enabled=" + std::to_string(oob_enabled) +
                " target=" + std::to_string(ntohl(target)) +
                " port=" + std::to_string(ntohs(port)));
        }
    }

    void measurement(bool enabled) override {
        require_open();
        check(enabled ? device_->startMeasurement() : device_->stopMeasurement());
    }

    void streaming(bool enabled) override {
        require_open();
        check(enabled ? device_->startStreaming() : device_->stopStreaming());
    }

    void close() noexcept override {
        try {
            device_->close();
        } catch (...) {
        }
    }

private:
    static void check(ldcp_sdk::error_t error) {
        if (error != ldcp_sdk::no_error) {
            throw std::runtime_error("LDCP failure code " +
                                     std::to_string(static_cast<int>(error)));
        }
    }

    void require_open() const {
        if (!device_->isOpened()) {
            throw std::runtime_error("control transport disconnected");
        }
    }

    std::string expected_model_;
    std::unique_ptr<ldcp_sdk::Device> device_;
    in_addr_t target_ = 0;
    in_port_t data_port_ = 0;
};

}
