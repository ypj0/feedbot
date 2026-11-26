#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <sstream>
#include <string>

namespace feedbot_imu_ros2 {

class CallbackGapHistogram {
public:
    static constexpr size_t bucket_count = 107;
    using Counts = std::array<uint64_t, bucket_count>;

    void observe(double seconds) noexcept {
        const auto milliseconds = seconds * 1000.0;
        size_t bucket = 0;
        if (milliseconds < 10.0) {
            bucket = static_cast<size_t>(std::floor(milliseconds * 10.0));
        } else if (milliseconds < 20.0) {
            bucket = 100;
        } else if (milliseconds < 50.0) {
            bucket = 101;
        } else if (milliseconds < 100.0) {
            bucket = 102;
        } else if (milliseconds < 200.0) {
            bucket = 103;
        } else if (milliseconds < 500.0) {
            bucket = 104;
        } else if (milliseconds < 1000.0) {
            bucket = 105;
        } else {
            bucket = 106;
        }
        ++counts_[bucket];
        ++sample_count_;
    }

    const Counts &counts() const noexcept { return counts_; }
    uint64_t sample_count() const noexcept { return sample_count_; }

    std::string counts_csv() const {
        std::ostringstream output;
        for (size_t index = 0; index < counts_.size(); ++index) {
            if (index != 0) output << ',';
            output << counts_[index];
        }
        return output.str();
    }

    void reset() noexcept {
        counts_.fill(0);
        sample_count_ = 0;
    }

private:
    Counts counts_{};
    uint64_t sample_count_ = 0;
};

}
