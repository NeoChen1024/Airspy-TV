#pragma once

#include <nlohmann/json.hpp>

#include <cmath>
#include <concepts>
#include <cstddef>
#include <istream>
#include <optional>
#include <ostream>
#include <span>
#include <stdexcept>
#include <string>

namespace airspy_tv {

class JsonlError : public std::runtime_error {
  public:
    JsonlError(std::string message, std::size_t line_number = 0);

    [[nodiscard]] std::size_t line_number() const noexcept;

  private:
    std::size_t line_number_{};
};

struct JsonlRecord {
    std::size_t line_number{};
    nlohmann::json value;
};

// Non-owning streaming JSONL reader. A final record without a trailing newline
// is accepted, but blank lines are rejected because each JSONL line must be a
// complete JSON value.
class JsonlReader {
  public:
    explicit JsonlReader(std::istream &stream) noexcept;

    [[nodiscard]] std::optional<JsonlRecord> read();
    [[nodiscard]] std::size_t line_number() const noexcept;

  private:
    std::istream *stream_{};
    std::size_t line_number_{};
};

// Non-owning streaming JSONL writer. The caller owns stream lifetime and flush
// policy. One compact JSON value plus '\n' is emitted for every write().
class JsonlWriter {
  public:
    explicit JsonlWriter(std::ostream &stream) noexcept;

    void write(const nlohmann::json &value);
    // Validate and serialize the complete batch before issuing one stream
    // write. std::vector<json> and std::array<json, N> both convert to span.
    void write_batch(std::span<const nlohmann::json> values);
    void flush();
    [[nodiscard]] std::size_t records_written() const noexcept;

  private:
    std::ostream *stream_{};
    std::size_t records_written_{};
};

// Report serializers use null for an unavailable floating-point measurement.
// Keeping this conversion explicit also lets JsonlWriter reject accidental
// NaN/Inf values that would otherwise be silently rewritten by a serializer.
template <std::floating_point T>
[[nodiscard]] nlohmann::json json_finite_or_null(const T value) {
    if (std::isfinite(value)) {
        return value;
    }
    return nullptr;
}

} // namespace airspy_tv
