#pragma once

#include <algorithm>
#include <cmath>

namespace ltr2 {

inline bool acceptable_receive_age(double age) {
    return std::isfinite(age) && age >= -0.02 && age <= 0.2;
}

inline bool fresh_input(double age_at_acceptance, double elapsed_monotonic) {
    return acceptable_receive_age(age_at_acceptance) &&
           std::isfinite(elapsed_monotonic) && elapsed_monotonic >= 0 &&
           std::max(0.0, age_at_acceptance) + elapsed_monotonic <= 0.3;
}

template <typename MonotonicClock, typename WallClock, typename Publish>
bool publish_if_fresh(double initial_age, double initial_monotonic, double receive_wall,
                      MonotonicClock monotonic_clock, WallClock wall_clock, Publish publish) {
    const double current_wall_age = wall_clock() - receive_wall;
    const double elapsed = monotonic_clock() - initial_monotonic;
    if (!acceptable_receive_age(initial_age) || !acceptable_receive_age(current_wall_age) ||
        !std::isfinite(elapsed) || elapsed < 0 ||
        std::max(0.0, initial_age) + elapsed > 0.2) {
        return false;
    }
    publish();
    return true;
}

}
