#include "airspy_tv/jsonl.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

using airspy_tv::json_finite_or_null;
using airspy_tv::JsonlError;
using airspy_tv::JsonlReader;
using airspy_tv::JsonlWriter;
using nlohmann::json;

void require(const bool condition, const std::string_view message) {
    if (!condition) {
        throw std::runtime_error(std::string{message});
    }
}

template <typename Function>
JsonlError expect_jsonl_error(Function &&function,
                              const std::string_view message) {
    try {
        std::forward<Function>(function)();
    } catch (const JsonlError &error) {
        return error;
    }
    throw std::runtime_error(std::string{message});
}

void test_round_trip_and_record_numbers() {
    std::ostringstream output;
    JsonlWriter writer(output);
    writer.write(json{{"record_type", "first"}, {"sequence", 1}});
    writer.write(json{{"message", "line one\nline two"}, {"ok", true}});
    writer.flush();

    require(writer.records_written() == 2, "writer record count is wrong");
    require(output.str().ends_with('\n'), "writer did not terminate JSONL");
    require(output.str().find("\nline two") == std::string::npos,
            "writer emitted a literal newline inside one JSON record");

    std::istringstream input(output.str());
    JsonlReader reader(input);
    const auto first = reader.read();
    const auto second = reader.read();
    if (!first.has_value()) {
        throw std::runtime_error("first record is missing");
    }
    if (!second.has_value()) {
        throw std::runtime_error("second record is missing");
    }
    require(first->line_number == 1, "first line number is wrong");
    require(first->value.at("record_type") == "first",
            "first value changed during round trip");
    require(second->line_number == 2, "second line number is wrong");
    require(second->value.at("message") == "line one\nline two",
            "escaped newline changed during round trip");
    require(!reader.read().has_value(), "reader did not stop at clean EOF");
    require(reader.line_number() == 2, "reader line count is wrong");
}

void test_final_record_without_newline() {
    std::istringstream input("{\"value\":7}");
    JsonlReader reader(input);
    const auto record = reader.read();
    require(record.has_value() && record->value.at("value") == 7,
            "reader rejected a valid final unterminated record");
    require(!reader.read().has_value(), "unterminated record was read twice");
}

void test_invalid_input_reports_line() {
    std::istringstream malformed("{\"ok\":true}\nnot-json\n");
    JsonlReader malformed_reader(malformed);
    require(malformed_reader.read().has_value(), "valid prefix was not read");
    const JsonlError parse_error =
        expect_jsonl_error([&] { static_cast<void>(malformed_reader.read()); },
                           "malformed JSON did not fail");
    require(parse_error.line_number() == 2,
            "parse error did not identify the malformed line");

    std::istringstream blank("{\"ok\":true}\n\n");
    JsonlReader blank_reader(blank);
    require(blank_reader.read().has_value(),
            "valid prefix before blank failed");
    const JsonlError blank_error =
        expect_jsonl_error([&] { static_cast<void>(blank_reader.read()); },
                           "blank JSONL record did not fail");
    require(blank_error.line_number() == 2,
            "blank-record error did not identify its line");
}

void test_non_finite_policy() {
    require(json_finite_or_null(1.25).is_number_float(),
            "finite helper changed a finite number");
    require(
        json_finite_or_null(std::numeric_limits<double>::infinity()).is_null(),
        "finite helper did not convert infinity to null");
    require(
        json_finite_or_null(std::numeric_limits<float>::quiet_NaN()).is_null(),
        "finite helper did not convert NaN to null");

    std::ostringstream output;
    JsonlWriter writer(output);
    const JsonlError error = expect_jsonl_error(
        [&] {
            writer.write(json{
                {"measurement", std::numeric_limits<double>::quiet_NaN()}});
        },
        "writer silently accepted a non-finite number");
    require(error.line_number() == 1,
            "non-finite writer error has the wrong record number");
    require(writer.records_written() == 0,
            "rejected record incremented the writer count");
    require(output.str().empty(), "rejected record wrote partial output");
}

void test_batch_write() {
    std::ostringstream output;
    JsonlWriter writer(output);
    const std::vector<json> vector_batch{json{{"sequence", 1}},
                                         json{{"sequence", 2}}};
    writer.write_batch(vector_batch);
    const std::array<json, 2> array_batch{json{{"sequence", 3}},
                                          json{{"sequence", 4}}};
    writer.write_batch(array_batch);
    writer.write_batch(std::span<const json>{});

    require(writer.records_written() == 4,
            "batch writes produced the wrong record count");
    std::istringstream input(output.str());
    JsonlReader reader(input);
    for (int sequence = 1; sequence <= 4; ++sequence) {
        const auto record = reader.read();
        require(record.has_value() && record->value.at("sequence") == sequence,
                "batch record order changed");
    }
    require(!reader.read().has_value(), "batch output has extra records");

    std::ostringstream rejected_output;
    JsonlWriter rejected_writer(rejected_output);
    const std::array<json, 3> invalid_batch{
        json{{"sequence", 1}},
        json{{"measurement", std::numeric_limits<double>::infinity()}},
        json{{"sequence", 3}}};
    const JsonlError error =
        expect_jsonl_error([&] { rejected_writer.write_batch(invalid_batch); },
                           "invalid batch did not fail");
    require(error.line_number() == 2,
            "batch error did not identify the invalid record");
    require(rejected_writer.records_written() == 0,
            "rejected batch incremented the record count");
    require(rejected_output.str().empty(),
            "validation failure wrote a partial JSONL batch");
}

void test_stream_failures_are_reported() {
    std::ostringstream output;
    output.setstate(std::ios::badbit);
    JsonlWriter writer(output);
    const JsonlError write_error =
        expect_jsonl_error([&] { writer.write(json{{"value", 1}}); },
                           "bad output stream did not fail");
    require(write_error.line_number() == 1,
            "write error has the wrong record number");

    std::istringstream input("{\"value\":1}\n");
    input.setstate(std::ios::badbit);
    JsonlReader reader(input);
    const JsonlError read_error =
        expect_jsonl_error([&] { static_cast<void>(reader.read()); },
                           "bad input stream did not fail");
    require(read_error.line_number() == 1,
            "read error has the wrong line number");
}

} // namespace

int main() {
    try {
        test_round_trip_and_record_numbers();
        test_final_record_without_newline();
        test_invalid_input_reports_line();
        test_non_finite_policy();
        test_batch_write();
        test_stream_failures_are_reported();
    } catch (const std::exception &exception) {
        std::cerr << "JSONL test failed: " << exception.what() << '\n';
        return EXIT_FAILURE;
    }
    std::cout << "JSONL tests passed\n";
    return EXIT_SUCCESS;
}
