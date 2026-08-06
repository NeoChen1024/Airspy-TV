#pragma once

#include <atomic>

namespace airspy_tv {

// Process-wide diagnostic logging switch. main() sets it from --debug before
// starting any receiver workers; relaxed access is sufficient because the
// flag does not publish any other state.
extern std::atomic_bool debug_enabled;

[[nodiscard]] inline bool is_debug_enabled() noexcept {
    return debug_enabled.load(std::memory_order_relaxed);
}

} // namespace airspy_tv
