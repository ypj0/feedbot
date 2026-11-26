#include "ltr2/lifecycle.hpp"

#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

struct TestFailure : std::runtime_error {
    using std::runtime_error::runtime_error;
};

void expect(bool condition, const std::string& message) {
    if (!condition) {
        throw TestFailure(message);
    }
}

template <typename Function>
void expect_logic_error(Function function, const std::string& message) {
    try {
        function();
    } catch (const std::logic_error&) {
        return;
    }
    throw TestFailure(message);
}

struct FakeControl final : ltr2::Control {
    std::vector<std::string> calls;
    std::string failure;
    std::function<void(bool)> measurement_hook;
    std::function<void(bool)> streaming_hook;

    void open() override {
        call("open");
    }

    void validate() override {
        call("validate");
    }

    void measurement(bool enabled) override {
        const std::string name = enabled ? "measurement_on" : "measurement_off";
        calls.push_back(name);
        if (measurement_hook) {
            measurement_hook(enabled);
        }
        fail(name);
    }

    void streaming(bool enabled) override {
        const std::string name = enabled ? "streaming_on" : "streaming_off";
        calls.push_back(name);
        if (streaming_hook) {
            streaming_hook(enabled);
        }
        fail(name);
    }

    void close() noexcept override {
        calls.push_back("close");
    }

    int count(const std::string& name) const {
        int total = 0;
        for (const std::string& call_name : calls) {
            total += call_name == name;
        }
        return total;
    }

private:
    void call(const std::string& name) {
        calls.push_back(name);
        fail(name);
    }

    void fail(const std::string& name) {
        if (failure == name) {
            throw std::runtime_error(name + " failure");
        }
    }
};

struct Fixture {
    double now = 1.0;
    FakeControl control;
    ltr2::Lifecycle lifecycle;

    explicit Fixture(double timeout = 5.0)
        : lifecycle(control, [this]() { return now; }, timeout) {
    }

    void configure() {
        lifecycle.claim();
        expect(lifecycle.configure(), "configuration failed");
        expect(lifecycle.state() == ltr2::State::Inactive, "not inactive after configure");
        expect(!lifecycle.ready(), "configuration made lifecycle ready");
        expect(control.calls == std::vector<std::string>{
                   "open", "validate", "streaming_off", "measurement_off"},
               "configuration did not confirm both stops after validation");
        control.calls.clear();
    }

    uint64_t activate() {
        expect(lifecycle.activate(), "activation failed");
        expect(lifecycle.state() == ltr2::State::Starting, "not starting after activate");
        expect(!lifecycle.ready(), "activation became ready without a frame");
        return lifecycle.epoch();
    }
};

void test_constructor_and_unowned_guards() {
    FakeControl control;
    expect_logic_error([&]() {
        ltr2::Lifecycle lifecycle(control, []() { return 1.0; });
        lifecycle.configure();
    }, "configure accepted without ownership");
    expect_logic_error([&]() {
        ltr2::Lifecycle lifecycle(control, []() { return 1.0; });
        lifecycle.stop(false);
    }, "stop accepted without ownership");
    expect_logic_error([&]() {
        ltr2::Lifecycle lifecycle(control, []() { return 1.0; });
        lifecycle.release();
    }, "release accepted without ownership");
    expect(control.calls.empty(), "unowned operations reached control");
    try {
        ltr2::Lifecycle lifecycle(control, ltr2::Clock{});
        throw TestFailure("empty clock accepted");
    } catch (const std::invalid_argument&) {
    }
    for (double timeout : {0.0, -1.0, 31.0, std::numeric_limits<double>::infinity()}) {
        try {
            ltr2::Lifecycle lifecycle(control, []() { return 1.0; }, timeout);
            throw TestFailure("invalid timeout accepted");
        } catch (const std::invalid_argument&) {
        }
    }
}

void test_configure_faults_and_recovery() {
    for (const std::string failure : {"open", "validate"}) {
        Fixture fixture;
        fixture.lifecycle.claim();
        fixture.control.failure = failure;
        expect(!fixture.lifecycle.configure(), failure + " fault accepted");
        expect(fixture.lifecycle.state() == ltr2::State::Fault, failure + " did not fault");
        expect(!fixture.lifecycle.ready(), failure + " left ready");
        expect(fixture.control.count("close") == 1, failure + " did not close");
        expect(fixture.control.count("measurement_off") == 0,
               failure + " issued unvalidated measurement write");
        expect(fixture.control.count("streaming_off") == 0,
               failure + " issued unvalidated streaming write");
        fixture.control.failure.clear();
        expect(fixture.lifecycle.configure(), failure + " recovery configure failed");
        expect(!fixture.lifecycle.ready(), failure + " recovery auto-activated");
        expect(fixture.lifecycle.release(), failure + " recovery release failed");
    }
}

void test_configure_confirms_stopped_state() {
    Fixture configured;
    configured.configure();
    expect(configured.lifecycle.release(), "configured release failed");
    expect(configured.control.calls == std::vector<std::string>{"close"},
           "confirmed inactive release performed unexpected commands");
    for (const std::string failure : {"streaming_off", "measurement_off"}) {
        Fixture fixture;
        fixture.lifecycle.claim();
        fixture.control.failure = failure;
        expect(!fixture.lifecycle.configure(), failure + " initial stop accepted");
        expect(fixture.control.calls == std::vector<std::string>{
                   "open", "validate", "streaming_off", "measurement_off", "close"},
               failure + " initial stop did not attempt both commands");
        expect(fixture.lifecycle.state() == ltr2::State::Fault,
               failure + " initial stop did not fault");
        expect(!fixture.lifecycle.ready(), failure + " initial stop left ready");
        expect_logic_error([&]() { fixture.lifecycle.activate(); },
                           failure + " allowed activation after failed configure");
        expect(!fixture.lifecycle.release(), failure + " release claimed stopped");
    }
}

void test_activation_boundary_and_generation() {
    Fixture fixture;
    fixture.configure();
    const uint64_t before_activation = fixture.lifecycle.epoch();
    bool callback_accepted = true;
    fixture.control.measurement_hook = [&](bool enabled) {
        if (enabled) {
            fixture.now = 1.5;
            callback_accepted = fixture.lifecycle.frame(
                fixture.lifecycle.epoch(), fixture.now, fixture.now);
        }
    };
    fixture.control.streaming_hook = [&](bool enabled) {
        if (enabled) {
            fixture.now = 2.0;
            callback_accepted = callback_accepted || fixture.lifecycle.frame(
                fixture.lifecycle.epoch(), fixture.now, fixture.now);
        }
    };
    const uint64_t active_epoch = fixture.activate();
    expect(active_epoch != before_activation, "activation did not advance generation");
    expect(!callback_accepted, "frame callback became ready before acknowledgements");
    expect(!fixture.lifecycle.frame(before_activation, 2.1, 2.1),
           "preactivation generation accepted");
    expect(!fixture.lifecycle.frame(active_epoch, 1.9, 2.0),
           "frame received before final acknowledgement accepted");
    fixture.now = 2.1;
    expect(fixture.lifecycle.frame(active_epoch, 2.1, 2.1), "fresh frame rejected");
    expect(fixture.lifecycle.ready(), "fresh frame did not establish readiness");
    const uint64_t ready_epoch = fixture.lifecycle.epoch();
    expect(fixture.lifecycle.stop(true), "sleep stop failed");
    expect(fixture.lifecycle.state() == ltr2::State::Sleeping, "sleep state missing");
    expect(!fixture.lifecycle.ready(), "sleep retained readiness");
    expect(!fixture.lifecycle.frame(ready_epoch, 2.2, 2.2), "stopped generation accepted");
    fixture.control.measurement_hook = {};
    fixture.control.streaming_hook = {};
    fixture.now = 3.0;
    const uint64_t wake_epoch = fixture.activate();
    expect(wake_epoch != ready_epoch, "wake reused generation");
    expect(!fixture.lifecycle.ready(), "wake restored past readiness");
    expect(fixture.lifecycle.frame(wake_epoch, 3.1, 3.1), "wake frame rejected");
    expect(fixture.lifecycle.release(), "active release failed");
    expect(fixture.lifecycle.state() == ltr2::State::Disconnected,
           "release did not disconnect");
}

void test_activation_failures_attempt_both_stops() {
    for (const std::string failure : {"measurement_on", "streaming_on"}) {
        Fixture fixture;
        fixture.configure();
        fixture.control.failure = failure;
        expect(!fixture.lifecycle.activate(), failure + " accepted");
        expect(fixture.lifecycle.state() == ltr2::State::Fault, failure + " did not fault");
        expect(fixture.control.count("streaming_off") == 1,
               failure + " skipped streaming stop");
        expect(fixture.control.count("measurement_off") == 1,
               failure + " skipped measurement stop");
        expect(fixture.control.count("close") == 1, failure + " did not close");
    }

    Fixture rollback;
    rollback.configure();
    rollback.control.streaming_hook = [&](bool enabled) {
        if (enabled) {
            rollback.now = 0.5;
        }
    };
    expect(!rollback.lifecycle.activate(), "activation clock rollback accepted");
    expect(rollback.lifecycle.state() == ltr2::State::Fault,
           "activation rollback did not fault");
    expect(rollback.control.count("streaming_off") == 1,
           "activation rollback skipped streaming stop");
    expect(rollback.control.count("measurement_off") == 1,
           "activation rollback skipped measurement stop");
}

void test_frame_rejection_and_clock_faults() {
    Fixture stale;
    stale.configure();
    const uint64_t stale_epoch = stale.activate();
    stale.now = 1.3;
    expect(!stale.lifecycle.frame(stale_epoch, 1.0, 1.3), "stale frame accepted");
    expect(stale.lifecycle.state() == ltr2::State::Starting,
           "stale frame changed starting state");

    Fixture future;
    future.configure();
    const uint64_t future_epoch = future.activate();
    expect(!future.lifecycle.frame(future_epoch, 1.1, 1.0), "future frame accepted");
    expect(future.lifecycle.state() == ltr2::State::Fault, "future frame did not fault");
    expect(future.control.count("streaming_off") == 1, "future fault skipped streaming stop");
    expect(future.control.count("measurement_off") == 1,
           "future fault skipped measurement stop");

    Fixture nonfinite;
    nonfinite.configure();
    const uint64_t nonfinite_epoch = nonfinite.activate();
    expect(!nonfinite.lifecycle.frame(nonfinite_epoch,
                                      std::numeric_limits<double>::quiet_NaN(), 1.0),
           "nonfinite receive clock accepted");
    expect(nonfinite.lifecycle.state() == ltr2::State::Fault,
           "nonfinite receive clock did not fault");

    Fixture rollback;
    rollback.configure();
    const uint64_t rollback_epoch = rollback.activate();
    expect(rollback.lifecycle.frame(rollback_epoch, 1.1, 1.1), "baseline frame rejected");
    expect(!rollback.lifecycle.frame(rollback_epoch, 1.05, 1.05), "clock rollback accepted");
    expect(rollback.lifecycle.state() == ltr2::State::Fault,
           "clock rollback did not fault");
}

void test_timeouts() {
    Fixture startup(0.5);
    startup.configure();
    startup.activate();
    startup.lifecycle.tick(1.5);
    expect(startup.lifecycle.state() == ltr2::State::Fault,
           "startup deadline did not fault");
    expect(startup.control.count("streaming_off") == 1,
           "startup timeout skipped streaming stop");
    expect(startup.control.count("measurement_off") == 1,
           "startup timeout skipped measurement stop");

    Fixture active;
    active.configure();
    const uint64_t active_epoch = active.activate();
    expect(active.lifecycle.frame(active_epoch, 1.1, 1.1), "active timeout setup failed");
    active.lifecycle.tick(1.401);
    expect(active.lifecycle.state() == ltr2::State::Fault, "active timeout did not fault");

    Fixture nonfinite;
    nonfinite.configure();
    nonfinite.activate();
    nonfinite.lifecycle.tick(std::numeric_limits<double>::infinity());
    expect(nonfinite.lifecycle.state() == ltr2::State::Fault,
           "nonfinite tick did not fault");
}

void test_partial_stop_and_release() {
    Fixture configured;
    configured.configure();
    expect(configured.lifecycle.stop(true), "configured stop failed");
    expect(configured.control.count("streaming_off") == 1 &&
           configured.control.count("measurement_off") == 1,
           "explicit stop did not confirm both hardware stops");
    for (const std::string failure : {"streaming_off", "measurement_off"}) {
        Fixture fixture;
        fixture.configure();
        fixture.activate();
        fixture.control.failure = failure;
        expect(!fixture.lifecycle.stop(false), failure + " partial stop reported success");
        expect(fixture.control.count("streaming_off") == 1,
               failure + " did not attempt streaming stop");
        expect(fixture.control.count("measurement_off") == 1,
               failure + " did not attempt measurement stop");
        expect(fixture.lifecycle.state() == ltr2::State::Fault,
               failure + " partial stop did not fault");
        expect(!fixture.lifecycle.ready(), failure + " partial stop retained readiness");
        expect(!fixture.lifecycle.release(), failure + " fault release reported stopped");
        expect(fixture.lifecycle.state() == ltr2::State::Disconnected,
               failure + " release did not disconnect");
    }
}

void test_loss_and_explicit_reactivation() {
    Fixture ownership;
    ownership.configure();
    const uint64_t old_epoch = ownership.activate();
    expect(ownership.lifecycle.frame(old_epoch, 1.1, 1.1), "ownership setup frame rejected");
    const int writes_before_loss = ownership.control.count("streaming_off") +
                                   ownership.control.count("measurement_off");
    ownership.lifecycle.ownership_lost();
    expect(!ownership.lifecycle.owned(), "ownership loss retained ownership");
    expect(ownership.lifecycle.state() == ltr2::State::Fault,
           "ownership loss pretended stopped");
    expect(!ownership.lifecycle.ready(), "ownership loss retained readiness");
    expect(ownership.control.count("streaming_off") +
               ownership.control.count("measurement_off") == writes_before_loss,
           "ownership loss issued unowned writes");
    expect(!ownership.lifecycle.frame(old_epoch, 1.2, 1.2),
           "ownership loss accepted old generation");
    ownership.lifecycle.claim();
    expect(ownership.lifecycle.configure(), "reclaimed configure failed");
    expect(!ownership.lifecycle.ready(), "reclaimed configure auto-activated");

    Fixture transport;
    transport.configure();
    const uint64_t transport_epoch = transport.activate();
    expect(transport.lifecycle.frame(transport_epoch, 1.1, 1.1),
           "transport setup frame rejected");
    transport.lifecycle.transport_lost();
    expect(transport.lifecycle.owned(), "transport loss discarded ownership");
    expect(transport.lifecycle.state() == ltr2::State::Fault,
           "transport loss did not fault");
    expect(!transport.lifecycle.frame(transport_epoch, 1.2, 1.2),
           "transport loss accepted old generation");
    expect(transport.lifecycle.configure(), "transport reconnect configure failed");
    expect(!transport.lifecycle.ready(), "transport reconnect auto-activated");
    transport.now = 2.0;
    const uint64_t reconnected_epoch = transport.activate();
    expect(transport.lifecycle.frame(reconnected_epoch, 2.1, 2.1),
           "explicit reconnect activation failed");
}

}

int main() {
    try {
        test_constructor_and_unowned_guards();
        test_configure_faults_and_recovery();
        test_configure_confirms_stopped_state();
        test_activation_boundary_and_generation();
        test_activation_failures_attempt_both_stops();
        test_frame_rejection_and_clock_faults();
        test_timeouts();
        test_partial_stop_and_release();
        test_loss_and_explicit_reactivation();
        std::cout << "PASS bounded LT-R2-B lifecycle synthetic tests\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
