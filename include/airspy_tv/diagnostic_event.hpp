#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <variant>

namespace airspy_tv {

enum class DiagnosticEventSeverity { info, warning };

using DiagnosticEventValue =
    std::variant<std::int64_t, std::uint64_t, double, bool, std::string>;
using DiagnosticEventFields =
    std::map<std::string, DiagnosticEventValue, std::less<>>;

struct DiagnosticEvent {
    std::string name;
    DiagnosticEventSeverity severity{DiagnosticEventSeverity::info};
    DiagnosticEventFields fields;
};

// Common processing components can publish typed diagnostics without knowing
// which receiver mode, report format, or human-readable renderer consumes
// them. The enabled predicate keeps field construction out of hot paths when
// neither structured telemetry nor debug rendering is active.
struct DiagnosticEventHandler {
    std::function<bool()> enabled;
    std::function<void(DiagnosticEvent)> emit;

    [[nodiscard]] bool is_enabled() const {
        return enabled && emit && enabled();
    }
};

} // namespace airspy_tv
