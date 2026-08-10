#pragma once

#include <algorithm>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <span>
#include <string>
#include <vector>

// Bounded producer/consumer storage with absolute stream positions. Queue
// synchronization remains with StreamDecoder::Impl; this type owns the ring's
// positional invariants and prevents a capacity change with retained data.
class AbsoluteSampleRing {
  public:
    explicit AbsoluteSampleRing(const std::size_t capacity)
        : samples_(capacity) {}

    [[nodiscard]] std::size_t size() const noexcept { return samples_.size(); }
    [[nodiscard]] std::size_t used() const noexcept {
        return static_cast<std::size_t>(write_position - read_position);
    }
    [[nodiscard]] bool empty() const noexcept {
        return read_position == write_position;
    }

    void resize(const std::size_t capacity) {
        if (!empty()) {
            throw std::logic_error("cannot resize a non-empty sample ring");
        }
        samples_.resize(capacity);
    }

    void reset() noexcept {
        read_position = 0;
        write_position = 0;
        closed = false;
    }

    [[nodiscard]] std::complex<float> *data() noexcept {
        return samples_.data();
    }
    [[nodiscard]] const std::complex<float> *data() const noexcept {
        return samples_.data();
    }
    [[nodiscard]] std::complex<float> &operator[](const std::size_t index) {
        return samples_[index];
    }
    [[nodiscard]] const std::complex<float> &
    operator[](const std::size_t index) const {
        return samples_[index];
    }

    void copy_absolute(const std::uint64_t position,
                       const std::span<std::complex<float>> output) const {
        if (output.size() > samples_.size() || position < read_position ||
            position > write_position ||
            output.size() > write_position - position) {
            throw std::out_of_range(
                "sample ring read is outside retained data: position=" +
                std::to_string(position) + ", size=" +
                std::to_string(output.size()) + ", retained=[" +
                std::to_string(read_position) + ", " +
                std::to_string(write_position) + ")");
        }
        if (output.empty()) {
            return;
        }
        const std::size_t read_index =
            static_cast<std::size_t>(position % samples_.size());
        const std::size_t first =
            std::min(output.size(), samples_.size() - read_index);
        std::copy_n(samples_.data() + read_index, first, output.data());
        if (first < output.size()) {
            std::copy_n(samples_.data(), output.size() - first,
                        output.data() + first);
        }
    }

    std::uint64_t write_position{};
    std::uint64_t read_position{};
    bool closed{};

  private:
    std::vector<std::complex<float>> samples_;
};
