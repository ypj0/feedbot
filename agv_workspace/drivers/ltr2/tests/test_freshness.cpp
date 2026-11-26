#include "ltr2/freshness.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

static void require(bool condition) {
    if (!condition) {
        throw std::runtime_error("freshness regression");
    }
}

int main() {
    try {
        require(ltr2::acceptable_receive_age(0.0));
        require(ltr2::acceptable_receive_age(0.2));
        require(ltr2::acceptable_receive_age(-0.02));
        require(!ltr2::acceptable_receive_age(0.201));
        require(!ltr2::acceptable_receive_age(-0.021));
        require(!ltr2::acceptable_receive_age(std::numeric_limits<double>::quiet_NaN()));
        require(!ltr2::acceptable_receive_age(std::numeric_limits<double>::infinity()));
        require(ltr2::fresh_input(0.2, 0.099));
        require(!ltr2::fresh_input(0.2, 0.101));
        require(!ltr2::fresh_input(0.2, 0.3));
        require(ltr2::fresh_input(0.0, 0.3));
        require(!ltr2::fresh_input(0.0, 0.301));
        require(!ltr2::fresh_input(0.0, -0.001));
        require(!ltr2::fresh_input(0.0, std::numeric_limits<double>::quiet_NaN()));
        require(ltr2::fresh_input(-0.02, 0.299));
        require(!ltr2::fresh_input(-0.02, 0.301));
        unsigned published = 0;
        double monotonic = 10.0;
        double wall = 100.0;
        const auto monotonic_clock = [&]() { return monotonic; };
        const auto wall_clock = [&]() { return wall; };
        const auto publish = [&]() { ++published; };
        require(ltr2::publish_if_fresh(0.0, 10.0, 100.0,
                                      monotonic_clock, wall_clock, publish));
        require(published == 1);
        monotonic = 11.0;
        wall = 101.0;
        require(!ltr2::publish_if_fresh(0.0, 10.0, 100.0,
                                       monotonic_clock, wall_clock, publish));
        require(published == 1);
        wall = 100.0;
        require(!ltr2::publish_if_fresh(0.0, 10.0, 100.0,
                                       monotonic_clock, wall_clock, publish));
        monotonic = 10.001;
        wall = 99.0;
        require(!ltr2::publish_if_fresh(0.0, 10.0, 100.0,
                                       monotonic_clock, wall_clock, publish));
        wall = 100.199;
        monotonic = 10.199;
        require(!ltr2::publish_if_fresh(0.01, 10.0, 100.0,
                                       monotonic_clock, wall_clock, publish));
        require(published == 1);
        monotonic = 10.0;
        wall = 100.0;
        const auto wall_clock_with_suspension = [&]() { monotonic = 11.0; return wall; };
        require(!ltr2::publish_if_fresh(0.0, 10.0, 100.0,
                                       monotonic_clock, wall_clock_with_suspension, publish));
        require(published == 1);
        std::cout << "PASS receive-age boundaries and accumulated freshness age\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
