#pragma once

#include <algorithm>
#include <array>
#include <cstring>
#include <string_view>

#ifdef __linux__
#include <sys/prctl.h>
#endif

namespace airspy_tv {

inline void set_current_thread_name(const std::string_view name) noexcept {
#ifdef __linux__
    // Linux limits task names to 15 bytes plus the terminating null.
    std::array<char, 16> buffer{};
    const std::size_t length = std::min(name.size(), buffer.size() - 1);
    std::memcpy(buffer.data(), name.data(), length);
    static_cast<void>(::prctl(PR_SET_NAME, buffer.data(), 0UL, 0UL, 0UL));
#else
    static_cast<void>(name);
#endif
}

} // namespace airspy_tv
