#pragma once

#include "ltr2/control.hpp"

#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace ltr2 {

using Clock = std::function<double()>;

enum class State {
    Disconnected,
    Inactive,
    Starting,
    Active,
    Sleeping,
    Fault
};

class Lifecycle {
public:
    explicit Lifecycle(Control& control, Clock monotonic_clock,
                       double startup_timeout = 5.0)
        : control_(control), clock_(std::move(monotonic_clock)),
          startup_timeout_(startup_timeout) {
        if (!clock_) {
            throw std::invalid_argument("monotonic clock required");
        }
        if (!std::isfinite(startup_timeout_) || startup_timeout_ <= 0.0 ||
            startup_timeout_ > 30.0) {
            throw std::invalid_argument("startup timeout must be in (0, 30]");
        }
    }

    State state() const noexcept {
        return state_;
    }

    const std::string& last_error() const noexcept {
        return last_error_;
    }

    bool owned() const noexcept {
        return owned_;
    }

    bool ready() const noexcept {
        return owned_ && state_ == State::Active;
    }

    uint64_t epoch() const noexcept {
        return generation_;
    }

    void claim() {
        if (owned_ || (state_ != State::Disconnected && state_ != State::Fault)) {
            throw std::logic_error("claim state");
        }
        owned_ = true;
    }

    bool configure() {
        require_owner();
        if (state_ != State::Disconnected && state_ != State::Fault) {
            throw std::logic_error("configure state");
        }
        invalidate_readiness();
        connected_ = false;
        try {
            control_.open();
            connected_ = true;
            control_.validate();
            if (!attempt_stop()) {
                throw std::runtime_error("initial stop failure");
            }
            state_ = State::Inactive;
            last_error_.clear();
            return true;
        } catch (const std::exception& error) {
            fail_closed(error.what(), false);
        } catch (...) {
            fail_closed("unknown configure failure", false);
        }
        return false;
    }

    bool activate() {
        require_owner();
        if (state_ != State::Inactive && state_ != State::Sleeping) {
            throw std::logic_error("activate state");
        }
        double started = 0.0;
        try {
            started = clock_();
        } catch (const std::exception& error) {
            fail_closed(error.what(), true);
            return false;
        } catch (...) {
            fail_closed("activation clock failure", true);
            return false;
        }
        if (!observe_now(started)) {
            fail_closed("activation clock invalid or rolled back", true);
            return false;
        }
        invalidate_readiness();
        state_ = State::Starting;
        try {
            control_.measurement(true);
            control_.streaming(true);
            const double acknowledged = clock_();
            if (!observe_now(acknowledged) || acknowledged < started ||
                !std::isfinite(acknowledged + startup_timeout_)) {
                throw std::runtime_error("activation clock invalid or rolled back");
            }
            acceptance_floor_ = acknowledged;
            deadline_ = acknowledged + startup_timeout_;
            acceptance_enabled_ = true;
            last_error_.clear();
            return true;
        } catch (const std::exception& error) {
            fail_closed(error.what(), true);
        } catch (...) {
            fail_closed("unknown activation failure", true);
        }
        return false;
    }

    bool frame(uint64_t frame_epoch, double received_monotonic, double now) {
        if (!owned_ || frame_epoch != generation_ ||
            (state_ != State::Starting && state_ != State::Active)) {
            return false;
        }
        if (!acceptance_enabled_) {
            return false;
        }
        if (!observe_now(now)) {
            fail_closed("frame clock invalid or rolled back", true);
            return false;
        }
        if (!std::isfinite(received_monotonic) || received_monotonic < 0.0 ||
            received_monotonic > now) {
            fail_closed("frame receive clock invalid", true);
            return false;
        }
        if ((state_ == State::Starting && now >= deadline_) ||
            (state_ == State::Active && now - last_frame_ >= active_timeout_)) {
            fail_closed("frame timeout", true);
            return false;
        }
        if (received_monotonic < acceptance_floor_ ||
            now - received_monotonic > receive_age_limit_ ||
            received_monotonic <= last_frame_) {
            return false;
        }
        last_frame_ = received_monotonic;
        state_ = State::Active;
        return true;
    }

    void tick(double now) {
        if (state_ != State::Starting && state_ != State::Active) {
            return;
        }
        if (!observe_now(now)) {
            fail_closed("tick clock invalid or rolled back", true);
            return;
        }
        if ((state_ == State::Starting && now >= deadline_) ||
            (state_ == State::Active && now - last_frame_ >= active_timeout_)) {
            fail_closed("data timeout", true);
        }
    }

    bool stop(bool sleep) {
        require_owner();
        if (state_ == State::Disconnected) {
            return true;
        }
        if (state_ == State::Sleeping) {
            state_ = sleep ? State::Sleeping : State::Inactive;
            invalidate_readiness();
            last_error_.clear();
            return true;
        }
        if (!connected_) {
            invalidate_readiness();
            state_ = State::Fault;
            return false;
        }
        invalidate_readiness();
        state_ = State::Fault;
        const bool stopped = attempt_stop();
        if (stopped) {
            state_ = sleep ? State::Sleeping : State::Inactive;
            last_error_.clear();
        } else {
            last_error_ = "stop failure";
            control_.close();
            connected_ = false;
        }
        return stopped;
    }

    bool release() {
        require_owner();
        bool stopped = true;
        if (state_ == State::Starting || state_ == State::Active) {
            stopped = stop(false);
        } else if (state_ == State::Fault) {
            stopped = false;
        }
        control_.close();
        connected_ = false;
        owned_ = false;
        invalidate_readiness();
        state_ = State::Disconnected;
        return stopped;
    }

    void ownership_lost() noexcept {
        owned_ = false;
        invalidate_readiness();
        state_ = State::Fault;
        connected_ = false;
        try {
            last_error_ = "exclusive ownership lost; stop state unknown";
        } catch (...) {
        }
        control_.close();
    }

    void transport_lost() noexcept {
        invalidate_readiness();
        state_ = State::Fault;
        connected_ = false;
        try {
            last_error_ = "control transport lost; stop state unknown";
        } catch (...) {
        }
        control_.close();
    }

private:
    void require_owner() const {
        if (!owned_) {
            throw std::logic_error("exclusive ownership absent");
        }
    }

    void invalidate_readiness() noexcept {
        ++generation_;
        acceptance_enabled_ = false;
        acceptance_floor_ = std::numeric_limits<double>::infinity();
        deadline_ = 0.0;
        last_frame_ = -1.0;
    }

    bool observe_now(double now) noexcept {
        if (!std::isfinite(now) || now < 0.0 ||
            (have_observed_time_ && now < last_observed_time_)) {
            return false;
        }
        last_observed_time_ = now;
        have_observed_time_ = true;
        return true;
    }

    bool attempt_stop() noexcept {
        bool stopped = true;
        try {
            control_.streaming(false);
        } catch (...) {
            stopped = false;
        }
        try {
            control_.measurement(false);
        } catch (...) {
            stopped = false;
        }
        return stopped;
    }

    void fail_closed(const std::string& error, bool stop_if_connected) {
        invalidate_readiness();
        state_ = State::Fault;
        last_error_ = error;
        if (stop_if_connected && connected_ && owned_) {
            attempt_stop();
        }
        control_.close();
        connected_ = false;
    }

    static constexpr double receive_age_limit_ = 0.2;
    static constexpr double active_timeout_ = 0.3;
    Control& control_;
    Clock clock_;
    double startup_timeout_;
    State state_ = State::Disconnected;
    std::string last_error_;
    bool owned_ = false;
    bool connected_ = false;
    bool acceptance_enabled_ = false;
    bool have_observed_time_ = false;
    uint64_t generation_ = 0;
    double acceptance_floor_ = std::numeric_limits<double>::infinity();
    double deadline_ = 0.0;
    double last_frame_ = -1.0;
    double last_observed_time_ = 0.0;
};

}
