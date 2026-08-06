#pragma once

#include <complex>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
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

    std::uint64_t write_position{};
    std::uint64_t read_position{};
    bool closed{};

  private:
    std::vector<std::complex<float>> samples_;
};
