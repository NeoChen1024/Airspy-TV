#include "telemetry_stream_router.hpp"

#include "airspy_tv/jsonl.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string_view>

namespace {

bool require(const bool condition, const std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
    }
    return condition;
}

bool test_multiple_stream_lifetimes() {
    const auto directory = std::filesystem::temp_directory_path() /
                           "airspy-tv-telemetry-router-test";
    std::error_code error;
    std::filesystem::remove_all(directory, error);
    std::filesystem::create_directories(directory);
    {
        airspy_tv::TelemetryStreamRouter router(
            directory,
            {{.key = "first", .path = "first.jsonl", .record_type = "first"},
             {.key = "second",
              .path = "second.jsonl",
              .record_type = "second"}});
        router.write("first", nlohmann::json{{"sequence", 1}});
        router.write("second", nlohmann::json{{"sequence", 2}});
        router.flush();
    }
    std::ifstream first_stream(directory / "first.jsonl");
    std::ifstream second_stream(directory / "second.jsonl");
    airspy_tv::JsonlReader first(first_stream);
    airspy_tv::JsonlReader second(second_stream);
    const auto first_record = first.read();
    const auto second_record = second.read();
    const bool result = require(first_record.has_value() &&
                                    first_record->value.at("sequence") == 1,
                                "first stream lost its writer lifetime") &&
                        require(second_record.has_value() &&
                                    second_record->value.at("sequence") == 2,
                                "second stream lost its writer lifetime");
    std::filesystem::remove_all(directory, error);
    return result;
}

} // namespace

int main() { return test_multiple_stream_lifetimes() ? 0 : 1; }
