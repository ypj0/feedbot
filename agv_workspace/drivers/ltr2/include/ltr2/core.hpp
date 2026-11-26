#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace ltr2 {

struct Block {
    uint16_t frame;
    uint8_t index;
    uint32_t stamp;
    std::array<uint16_t, 256> ranges;
    std::array<uint16_t, 256> intensities;
};

namespace detail {

inline uint16_t little_u16(const uint8_t* data) {
    return static_cast<uint16_t>(data[0]) |
           static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8U);
}

inline uint32_t little_u32(const uint8_t* data) {
    return static_cast<uint32_t>(little_u16(data)) |
           (static_cast<uint32_t>(little_u16(data + 2)) << 16U);
}

inline uint16_t packet_crc(const uint8_t* data, std::size_t size) {
    uint16_t crc = 0xffffU;
    for (std::size_t offset = 0; offset < size; ++offset) {
        const uint8_t value = offset == 12U || offset == 13U ? 0U : data[offset];
        crc ^= static_cast<uint16_t>(value) << 8U;
        for (unsigned bit_index = 0; bit_index < 8U; ++bit_index) {
            crc = (crc & 0x8000U) != 0U
                      ? static_cast<uint16_t>((crc << 1U) ^ 0x1021U)
                      : static_cast<uint16_t>(crc << 1U);
        }
    }
    return crc;
}

inline bool forward_frame(uint16_t candidate, uint16_t reference) {
    constexpr uint16_t maximum_frame_advance = 1024U;
    const uint16_t delta = static_cast<uint16_t>(candidate - reference);
    return delta != 0U && delta <= maximum_frame_advance;
}

inline bool forward_device_stamp(uint32_t candidate, uint32_t reference, uint32_t& delta) {
    delta = static_cast<uint32_t>(candidate - reference);
    return delta != 0U && delta < 0x80000000U;
}

}

inline Block decode(const uint8_t* data, std::size_t size) {
    constexpr std::size_t packet_size = 1040U;
    constexpr uint16_t block_length = 256U;
    constexpr uint8_t block_count = 8U;
    constexpr uint16_t required_flags = 0x0101U;
    if (data == nullptr || size != packet_size) {
        throw std::runtime_error("LT-R2-B OOB length");
    }
    if (detail::little_u16(data) != 0xffffU) {
        throw std::runtime_error("LT-R2-B OOB magic");
    }
    if (data[4] >= block_count || data[5] != block_count ||
        detail::little_u16(data + 6) != block_length ||
        detail::little_u16(data + 14) != required_flags) {
        throw std::runtime_error("LT-R2-B OOB layout");
    }
    if (detail::packet_crc(data, size) != detail::little_u16(data + 12)) {
        throw std::runtime_error("LT-R2-B OOB CRC");
    }

    Block block{};
    block.frame = detail::little_u16(data + 2);
    block.index = data[4];
    block.stamp = detail::little_u32(data + 8);
    constexpr std::size_t ranges_offset = 16U;
    constexpr std::size_t intensities_offset = ranges_offset + block_length * 2U;
    for (std::size_t sample_index = 0; sample_index < block_length; ++sample_index) {
        block.ranges[sample_index] =
            detail::little_u16(data + ranges_offset + sample_index * 2U);
        block.intensities[sample_index] =
            detail::little_u16(data + intensities_offset + sample_index * 2U);
    }
    return block;
}

struct Scan {
    uint16_t frame;
    double stamp;
    double scan_time;
    double time_increment;
    double angle_min;
    double angle_max;
    double angle_increment;
    std::vector<float> ranges;
    std::vector<float> intensities;
};

class Assembler {
public:
    bool add(const Block& block, double receive_wall, double receive_monotonic, Scan& scan) {
        if (!std::isfinite(receive_wall) || receive_wall <= 0.0 ||
            !std::isfinite(receive_monotonic)) {
            invalidate();
            return false;
        }
        if (have_host_clock_ &&
            (receive_wall < last_receive_wall_ || receive_monotonic < last_receive_monotonic_)) {
            invalidate();
            last_receive_wall_ = receive_wall;
            last_receive_monotonic_ = receive_monotonic;
            return false;
        }
        have_host_clock_ = true;
        last_receive_wall_ = receive_wall;
        last_receive_monotonic_ = receive_monotonic;

        if (block.index >= block_count_) {
            invalidate();
            return false;
        }

        if (active_) {
            if (block.frame != current_frame_) {
                const bool newer_frame = detail::forward_frame(block.frame, current_frame_);
                discard_active();
                if (!newer_frame) {
                    return false;
                }
                latest_frame_ = block.frame;
                have_latest_frame_ = true;
                if (block.index != 0U) {
                    return false;
                }
                if (!start(block, receive_wall, receive_monotonic)) {
                    return false;
                }
                return false;
            }
            if (block.index != next_index_) {
                discard_active();
                return false;
            }
        } else {
            if (block.index != 0U) {
                if (!have_latest_frame_) {
                    latest_frame_ = block.frame;
                    have_latest_frame_ = true;
                } else if (detail::forward_frame(block.frame, latest_frame_)) {
                    latest_frame_ = block.frame;
                }
                return false;
            }
            if (have_latest_frame_ && !detail::forward_frame(block.frame, latest_frame_)) {
                return false;
            }
            if (!start(block, receive_wall, receive_monotonic)) {
                return false;
            }
            return false;
        }

        uint32_t device_delta = 0U;
        if (!detail::forward_device_stamp(block.stamp, previous_device_stamp_, device_delta) ||
            device_delta > maximum_frame_duration_microseconds_ ||
            device_elapsed_microseconds_ + device_delta > maximum_frame_duration_microseconds_) {
            discard_active();
            return false;
        }
        if (receive_wall - first_receive_wall_ > maximum_frame_duration_seconds_ ||
            receive_monotonic - first_receive_monotonic_ > maximum_frame_duration_seconds_) {
            discard_active();
            return false;
        }

        copy_block(block);
        previous_device_stamp_ = block.stamp;
        last_device_stamp_ = block.stamp;
        device_elapsed_microseconds_ += device_delta;
        ++next_index_;
        if (next_index_ != block_count_) {
            return false;
        }

        const double scan_time = static_cast<double>(device_elapsed_microseconds_) * 1.0e-6 *
                                 static_cast<double>(block_count_) /
                                 static_cast<double>(block_count_ - 1U);
        const double host_wall_duration = receive_wall - first_receive_wall_;
        const double host_monotonic_duration = receive_monotonic - first_receive_monotonic_;
        if (scan_time < minimum_scan_time_seconds_ || scan_time > maximum_scan_time_seconds_ ||
            host_wall_duration <= 0.0 || host_monotonic_duration <= 0.0 ||
            host_wall_duration > maximum_frame_duration_seconds_ ||
            host_monotonic_duration > maximum_frame_duration_seconds_) {
            discard_active();
            return false;
        }

        const double estimated_stamp =
            first_receive_wall_ - scan_time / static_cast<double>(block_count_) +
            static_cast<double>(crop_start_index_) *
                (scan_time / static_cast<double>(beam_count_));
        if (!std::isfinite(estimated_stamp) || estimated_stamp <= 0.0 ||
            (have_emitted_stamp_ && estimated_stamp <= last_emitted_stamp_)) {
            discard_active();
            return false;
        }

        Scan completed{};
        completed.frame = current_frame_;
        completed.scan_time = scan_time;
        completed.time_increment = scan_time / static_cast<double>(beam_count_);
        completed.angle_increment = two_pi_ / static_cast<double>(beam_count_);
        completed.angle_min = static_cast<double>(crop_low_offset_) * completed.angle_increment;
        completed.angle_max = static_cast<double>(crop_high_offset_) * completed.angle_increment;
        completed.stamp = estimated_stamp;
        completed.ranges.reserve(crop_sample_count_);
        completed.intensities.reserve(crop_sample_count_);
        for (std::size_t beam_index = crop_start_index_; beam_index <= crop_end_index_; ++beam_index) {
            const uint16_t raw_range = ranges_[beam_index];
            if (raw_range == 0U) {
                completed.ranges.push_back(std::numeric_limits<float>::infinity());
                completed.intensities.push_back(0.0F);
            } else {
                completed.ranges.push_back(static_cast<float>(raw_range) * range_unit_metres_);
                completed.intensities.push_back(static_cast<float>(intensities_[beam_index]));
            }
        }

        active_ = false;
        have_emitted_stamp_ = true;
        last_emitted_stamp_ = estimated_stamp;
        scan = std::move(completed);
        return true;
    }

    void invalidate() {
        discard_active();
    }

    uint64_t discarded() const {
        return discarded_;
    }

private:
    static constexpr uint8_t block_count_ = 8U;
    static constexpr std::size_t samples_per_block_ = 256U;
    static constexpr std::size_t beam_count_ = block_count_ * samples_per_block_;
    static constexpr double two_pi_ = 6.283185307179586476925286766559;
    static constexpr double minimum_scan_time_seconds_ = 0.025;
    static constexpr double maximum_scan_time_seconds_ = 0.150;
    static constexpr double maximum_frame_duration_seconds_ = 0.200;
    static constexpr uint64_t maximum_frame_duration_microseconds_ = 200000U;
    static constexpr uint32_t maximum_interframe_device_delta_microseconds_ = 153600000U;
    static constexpr float range_unit_metres_ = 0.002F;
    static constexpr int crop_low_offset_ = -537;
    static constexpr int crop_high_offset_ = 537;
    static constexpr std::size_t crop_start_index_ = static_cast<std::size_t>(
        static_cast<int>(beam_count_ / 2U) + crop_low_offset_);
    static constexpr std::size_t crop_end_index_ = static_cast<std::size_t>(
        static_cast<int>(beam_count_ / 2U) + crop_high_offset_);
    static constexpr std::size_t crop_sample_count_ = crop_end_index_ - crop_start_index_ + 1U;

    bool start(const Block& block, double receive_wall, double receive_monotonic) {
        uint32_t device_delta = 0U;
        if (have_device_stamp_ &&
            (!detail::forward_device_stamp(block.stamp, last_device_stamp_, device_delta) ||
             device_delta > maximum_interframe_device_delta_microseconds_)) {
            return false;
        }
        active_ = true;
        current_frame_ = block.frame;
        latest_frame_ = block.frame;
        have_latest_frame_ = true;
        next_index_ = 1U;
        previous_device_stamp_ = block.stamp;
        last_device_stamp_ = block.stamp;
        have_device_stamp_ = true;
        device_elapsed_microseconds_ = 0U;
        first_receive_wall_ = receive_wall;
        first_receive_monotonic_ = receive_monotonic;
        copy_block(block);
        return true;
    }

    void copy_block(const Block& block) {
        const std::size_t destination_offset = static_cast<std::size_t>(block.index) * samples_per_block_;
        for (std::size_t sample_index = 0; sample_index < samples_per_block_; ++sample_index) {
            ranges_[destination_offset + sample_index] = block.ranges[sample_index];
            intensities_[destination_offset + sample_index] = block.intensities[sample_index];
        }
    }

    void discard_active() {
        if (active_) {
            active_ = false;
            ++discarded_;
        }
    }

    bool active_ = false;
    bool have_latest_frame_ = false;
    bool have_host_clock_ = false;
    bool have_device_stamp_ = false;
    bool have_emitted_stamp_ = false;
    uint16_t current_frame_ = 0U;
    uint16_t latest_frame_ = 0U;
    uint8_t next_index_ = 0U;
    uint32_t previous_device_stamp_ = 0U;
    uint32_t last_device_stamp_ = 0U;
    uint64_t device_elapsed_microseconds_ = 0U;
    double first_receive_wall_ = 0.0;
    double first_receive_monotonic_ = 0.0;
    double last_receive_wall_ = 0.0;
    double last_receive_monotonic_ = 0.0;
    double last_emitted_stamp_ = 0.0;
    uint64_t discarded_ = 0U;
    std::array<uint16_t, beam_count_> ranges_{};
    std::array<uint16_t, beam_count_> intensities_{};
};

}
