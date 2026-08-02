#include "airspy_tv/dvbt/decoder.hpp"
#include "airspy_tv/dvbt/soft_demapper.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <span>
#include <string_view>
#include <vector>

namespace {

using airspy_tv::dvbt::CodeRate;
using airspy_tv::dvbt::Constellation;
using airspy_tv::dvbt::Decoder;
using airspy_tv::dvbt::DecoderParameters;
using airspy_tv::dvbt::MaxLogDemapper;
using airspy_tv::dvbt::TransmissionMode;

constexpr std::size_t carriers_per_symbol = 6048;

void normalize_symbol(std::span<std::complex<float>> carriers,
                      const std::span<const std::complex<float>> points) {
    for (int iteration = 0; iteration < 2; ++iteration) {
        std::complex<double> numerator{};
        double denominator = 0.0;
        for (const auto carrier : carriers) {
            auto nearest = points.front();
            float distance = std::numeric_limits<float>::infinity();
            for (const auto point : points) {
                const float candidate = std::norm(carrier - point);
                if (candidate < distance) {
                    distance = candidate;
                    nearest = point;
                }
            }
            numerator += std::conj(static_cast<std::complex<double>>(nearest)) *
                         static_cast<std::complex<double>>(carrier);
            denominator += std::norm(nearest);
        }
        if (denominator <= 0.0) {
            return;
        }
        const auto gain =
            static_cast<std::complex<float>>(numerator / denominator);
        if (std::abs(gain) <= 1.0e-6F) {
            return;
        }
        for (auto &carrier : carriers) {
            carrier /= gain;
        }
    }
}

std::vector<float>
estimate_reliability(const std::span<const std::complex<float>> carriers,
                     const std::span<const std::complex<float>> points) {
    std::vector<float> errors;
    errors.reserve(carriers.size());
    for (const auto carrier : carriers) {
        float nearest = std::numeric_limits<float>::infinity();
        for (const auto point : points) {
            nearest = std::min(nearest, std::norm(carrier - point));
        }
        errors.push_back(nearest);
    }
    auto middle =
        errors.begin() + static_cast<std::ptrdiff_t>(errors.size() / 2);
    std::ranges::nth_element(errors, middle);
    const float noise_variance = std::max(*middle / std::log(2.0F), 1.0e-4F);
    return std::vector<float>(carriers.size(), 1.0F / noise_variance);
}

int decode(const std::filesystem::path &source,
           const std::filesystem::path &destination,
           const std::size_t first_symbol) {
    std::ifstream input(source, std::ios::binary);
    std::ofstream output(destination, std::ios::binary | std::ios::trunc);
    if (!input || !output) {
        std::cerr << "failed to open input or output file\n";
        return 1;
    }

    const DecoderParameters parameters{
        TransmissionMode::k8, Constellation::qam64, CodeRate::rate_2_3};
    Decoder decoder{parameters};
    const MaxLogDemapper reference{parameters.constellation};
    std::vector<std::complex<float>> carriers(carriers_per_symbol);
    std::size_t symbol_count = 0;
    std::uint64_t output_bytes = 0;
    while (input.read(reinterpret_cast<char *>(carriers.data()),
                      static_cast<std::streamsize>(carriers.size() *
                                                   sizeof(carriers.front())))) {
        normalize_symbol(carriers, reference.constellation_points());
        const auto reliability =
            estimate_reliability(carriers, reference.constellation_points());
        const auto transport_stream = decoder.process_symbol(
            carriers, reliability, (first_symbol + symbol_count) % 68);
        output.write(reinterpret_cast<const char *>(transport_stream.data()),
                     static_cast<std::streamsize>(transport_stream.size()));
        output_bytes += transport_stream.size();
        ++symbol_count;
        if ((symbol_count % 100) == 0) {
            std::cerr << "decoded " << symbol_count << " symbols, "
                      << output_bytes << " TS bytes\n";
        }
    }

    const auto stats = decoder.stats();
    std::cerr << "symbols=" << symbol_count << ", output=" << output_bytes
              << " bytes, RS=" << stats.rs_packets
              << ", RS failures=" << stats.rs_uncorrectable_packets
              << ", TS packets=" << stats.ts_packets << '\n';
    return output_bytes == 0 ? 2 : 0;
}

} // namespace

int main(const int argc, const char *const argv[]) {
    if (argc < 3 || argc > 4) {
        std::cerr << "usage: airspy-tv-dvbt-equalized INPUT.cfile OUTPUT.ts "
                     "[FIRST_SYMBOL]\n";
        return 1;
    }
    std::size_t first_symbol = 0;
    if (argc == 4) {
        try {
            first_symbol = static_cast<std::size_t>(std::stoul(argv[3]));
        } catch (...) {
            std::cerr << "invalid first symbol\n";
            return 1;
        }
    }
    return decode(argv[1], argv[2], first_symbol);
}
