#include "feedbot_imu_ros2/callback_gap_histogram.hpp"

#include <cassert>
#include <numeric>

int main() {
    feedbot_imu_ros2::CallbackGapHistogram histogram;
    histogram.observe(0.0);
    histogram.observe(0.000099999);
    histogram.observe(0.0001);
    histogram.observe(0.009999999);
    histogram.observe(0.010);
    histogram.observe(0.020);
    histogram.observe(0.050);
    histogram.observe(0.100);
    histogram.observe(0.200);
    histogram.observe(0.500);
    histogram.observe(1.000);

    const auto &counts = histogram.counts();
    assert(counts.size() == 107);
    assert(counts[0] == 2);
    assert(counts[1] == 1);
    assert(counts[99] == 1);
    assert(counts[100] == 1);
    assert(counts[101] == 1);
    assert(counts[102] == 1);
    assert(counts[103] == 1);
    assert(counts[104] == 1);
    assert(counts[105] == 1);
    assert(counts[106] == 1);
    assert(histogram.sample_count() == 11);
    assert(std::accumulate(counts.begin(), counts.end(), uint64_t{0}) == histogram.sample_count());

    histogram.reset();
    assert(histogram.sample_count() == 0);
    assert(std::accumulate(histogram.counts().begin(), histogram.counts().end(), uint64_t{0}) == 0);
}
