#pragma once

#include "decode_report.hpp"

#include <memory>

namespace airspy_tv {

[[nodiscard]] std::unique_ptr<DecodeReport> make_dvbt_decode_report();

} // namespace airspy_tv
