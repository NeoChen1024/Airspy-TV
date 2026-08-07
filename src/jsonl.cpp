#include "airspy_tv/jsonl.hpp"

#include <limits>
#include <sstream>
#include <string_view>
#include <utility>

namespace airspy_tv {
namespace {

[[nodiscard]] std::string with_line(const std::string_view message,
                                    const std::size_t line_number) {
    if (line_number == 0) {
        return std::string{message};
    }
    std::ostringstream formatted;
    formatted << message << " at JSONL line " << line_number;
    return formatted.str();
}

[[nodiscard]] bool contains_non_finite(const nlohmann::json &value) {
    if (value.is_number_float()) {
        return !std::isfinite(value.get<double>());
    }
    if (value.is_array()) {
        for (const auto &element : value) {
            if (contains_non_finite(element)) {
                return true;
            }
        }
    } else if (value.is_object()) {
        for (const auto &[key, element] : value.items()) {
            static_cast<void>(key);
            if (contains_non_finite(element)) {
                return true;
            }
        }
    }
    return false;
}

[[nodiscard]] std::string serialize_record(const nlohmann::json &value,
                                           const std::size_t record_number) {
    if (contains_non_finite(value)) {
        throw JsonlError("JSONL record contains NaN or infinity; convert the "
                         "measurement to null explicitly",
                         record_number);
    }

    try {
        return value.dump();
    } catch (const nlohmann::json::exception &exception) {
        throw JsonlError(std::string{"Unable to serialize JSON: "} +
                             exception.what(),
                         record_number);
    }
}

} // namespace

JsonlError::JsonlError(const std::string &message,
                       const std::size_t line_number)
    : std::runtime_error(with_line(message, line_number)),
      line_number_(line_number) {}

std::size_t JsonlError::line_number() const noexcept { return line_number_; }

JsonlReader::JsonlReader(std::istream &stream) noexcept : stream_(&stream) {}

std::optional<JsonlRecord> JsonlReader::read() {
    std::string line;
    if (!std::getline(*stream_, line)) {
        if (stream_->bad()) {
            throw JsonlError("Failed to read JSONL stream", line_number_ + 1);
        }
        if (stream_->eof()) {
            return std::nullopt;
        }
        throw JsonlError("Failed to read JSONL stream", line_number_ + 1);
    }
    ++line_number_;
    if (line.empty()) {
        throw JsonlError("Empty records are not valid JSONL", line_number_);
    }

    try {
        return JsonlRecord{.line_number = line_number_,
                           .value = nlohmann::json::parse(line)};
    } catch (const nlohmann::json::parse_error &exception) {
        throw JsonlError(std::string{"Invalid JSON: "} + exception.what(),
                         line_number_);
    }
}

std::size_t JsonlReader::line_number() const noexcept { return line_number_; }

JsonlWriter::JsonlWriter(std::ostream &stream) noexcept : stream_(&stream) {}

void JsonlWriter::write(const nlohmann::json &value) {
    const std::size_t record_number = records_written_ + 1;
    const std::string serialized = serialize_record(value, record_number);
    if (serialized.size() >
        static_cast<std::size_t>(std::numeric_limits<std::streamsize>::max())) {
        throw JsonlError("Serialized JSONL record is too large", record_number);
    }

    stream_->write(serialized.data(),
                   static_cast<std::streamsize>(serialized.size()));
    stream_->put('\n');
    if (!*stream_) {
        throw JsonlError("Failed to write JSONL stream", record_number);
    }
    ++records_written_;
}

void JsonlWriter::write_batch(const std::span<const nlohmann::json> values) {
    if (values.empty()) {
        return;
    }

    std::string serialized_batch;
    for (std::size_t index = 0; index < values.size(); ++index) {
        const std::size_t record_number = records_written_ + index + 1;
        std::string const serialized =
            serialize_record(values[index], record_number);
        if (serialized.size() == std::numeric_limits<std::size_t>::max() ||
            serialized_batch.size() > std::numeric_limits<std::size_t>::max() -
                                          serialized.size() - 1) {
            throw JsonlError("Serialized JSONL batch is too large",
                             record_number);
        }
        serialized_batch.append(serialized);
        serialized_batch.push_back('\n');
    }
    if (serialized_batch.size() >
        static_cast<std::size_t>(std::numeric_limits<std::streamsize>::max())) {
        throw JsonlError("Serialized JSONL batch is too large",
                         records_written_ + values.size());
    }

    stream_->write(serialized_batch.data(),
                   static_cast<std::streamsize>(serialized_batch.size()));
    if (!*stream_) {
        throw JsonlError("Failed to write JSONL stream", records_written_ + 1);
    }
    records_written_ += values.size();
}

void JsonlWriter::flush() {
    stream_->flush();
    if (!*stream_) {
        throw JsonlError("Failed to flush JSONL stream", records_written_);
    }
}

std::size_t JsonlWriter::records_written() const noexcept {
    return records_written_;
}

} // namespace airspy_tv
