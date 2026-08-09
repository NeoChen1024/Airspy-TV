#include "airspy_tv/dvbt/decoder.hpp"

#include <chrono>
#include <complex>
#include <cstddef>
#include <memory>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace airspy_tv::dvbt {

struct Decoder::Impl {
    explicit Impl(const DecoderParameters selected_parameters)
        : parameters(selected_parameters), demapper(parameters.constellation),
          symbol_deinterleaver(parameters.mode),
          transport_decoder(parameters.code_rate, parameters.viterbi_workers) {
        const std::size_t metric_count =
            payload_carrier_count(parameters.mode) *
            bits_per_symbol(parameters.constellation);
        demapped.resize(metric_count);
        symbol_metrics.resize(metric_count);
        bit_metrics.resize(metric_count);
    }

    [[nodiscard]] std::vector<std::uint8_t> process_symbol(
        const std::span<const std::complex<float>> equalized_carriers,
        const std::span<const float> reliability,
        const std::size_t symbol_index) {
        const std::size_t expected_carriers =
            payload_carrier_count(parameters.mode);
        if (equalized_carriers.size() != expected_carriers ||
            reliability.size() != expected_carriers) {
            throw std::invalid_argument("DVB-T payload carrier count mismatch");
        }

        const std::size_t bits = bits_per_symbol(parameters.constellation);
        const auto demap_started_at = std::chrono::steady_clock::now();
        demapper.demap(equalized_carriers, reliability, demapped);
        const auto deinterleave_started_at = std::chrono::steady_clock::now();
        symbol_deinterleaver.process(demapped, bits, symbol_index,
                                     symbol_metrics);
        bit_deinterleave(symbol_metrics, bits, bit_metrics);
        const auto transport_started_at = std::chrono::steady_clock::now();
        auto output = transport_decoder.process(bit_metrics);
        const auto finished_at = std::chrono::steady_clock::now();
        timing.demap_time_ms += std::chrono::duration<double, std::milli>(
                                    deinterleave_started_at - demap_started_at)
                                    .count();
        timing.deinterleave_time_ms +=
            std::chrono::duration<double, std::milli>(transport_started_at -
                                                      deinterleave_started_at)
                .count();
        timing.transport_time_ms += std::chrono::duration<double, std::milli>(
                                        finished_at - transport_started_at)
                                        .count();
        return output;
    }

    [[nodiscard]] std::vector<std::uint8_t>
    process_metrics(const std::span<const float> punctured_llrs) {
        if (punctured_llrs.size() != bit_metrics.size()) {
            throw std::invalid_argument(
                "DVB-T punctured metric count mismatch");
        }
        const auto started_at = std::chrono::steady_clock::now();
        auto output = transport_decoder.process(punctured_llrs);
        timing.transport_time_ms +=
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started_at)
                .count();
        return output;
    }

    [[nodiscard]] std::vector<std::uint8_t>
    process_soft_metrics(const std::span<const std::uint8_t> mother_metrics) {
        std::vector<std::uint8_t> output;
        process_soft_metrics(mother_metrics, output);
        return output;
    }

    void
    process_soft_metrics(const std::span<const std::uint8_t> mother_metrics,
                         std::vector<std::uint8_t> &output) {
        const std::size_t expected =
            depunctured_size(bit_metrics.size(), parameters.code_rate);
        if (mother_metrics.size() != expected) {
            throw std::invalid_argument(
                "DVB-T mother-code metric count mismatch");
        }
        const auto started_at = std::chrono::steady_clock::now();
        transport_decoder.process_soft(mother_metrics, output);
        timing.transport_time_ms +=
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started_at)
                .count();
    }

    void flush(std::vector<std::uint8_t> &output) {
        const auto started_at = std::chrono::steady_clock::now();
        transport_decoder.flush(output);
        timing.transport_time_ms +=
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started_at)
                .count();
    }

    DecoderParameters parameters;
    MaxLogDemapper demapper;
    SymbolDeinterleaver symbol_deinterleaver;
    TransportDecoder transport_decoder;
    std::vector<float> demapped;
    std::vector<float> symbol_metrics;
    std::vector<float> bit_metrics;
    DecoderTiming timing;
};

Decoder::Decoder(const DecoderParameters parameters)
    : impl_(std::make_unique<Impl>(parameters)) {}

Decoder::~Decoder() noexcept = default;
Decoder::Decoder(Decoder &&) noexcept = default;
Decoder &Decoder::operator=(Decoder &&) noexcept = default;

void Decoder::reset() {
    impl_->transport_decoder.reset();
    impl_->timing = {};
}

void Decoder::set_detailed_timing_enabled(const bool enabled) noexcept {
    impl_->transport_decoder.set_detailed_timing_enabled(enabled);
}

void Decoder::set_diagnostic_handler(DiagnosticEventHandler handler) {
    impl_->transport_decoder.set_diagnostic_handler(std::move(handler));
}

std::vector<std::uint8_t> Decoder::process_symbol(
    const std::span<const std::complex<float>> equalized_carriers,
    const std::span<const float> reliability, const std::size_t symbol_index) {
    return impl_->process_symbol(equalized_carriers, reliability, symbol_index);
}

std::vector<std::uint8_t>
Decoder::process_metrics(const std::span<const float> punctured_llrs) {
    return impl_->process_metrics(punctured_llrs);
}

std::vector<std::uint8_t> Decoder::process_soft_metrics(
    const std::span<const std::uint8_t> mother_metrics) {
    return impl_->process_soft_metrics(mother_metrics);
}

void Decoder::process_soft_metrics(
    const std::span<const std::uint8_t> mother_metrics,
    std::vector<std::uint8_t> &output) {
    impl_->process_soft_metrics(mother_metrics, output);
}

DecoderParameters Decoder::parameters() const noexcept {
    return impl_->parameters;
}

DecoderTiming Decoder::timing() const noexcept {
    auto result = impl_->timing;
    result.transport = impl_->transport_decoder.timing();
    return result;
}

TransportDecoderStats Decoder::stats() const {
    return impl_->transport_decoder.stats();
}

std::vector<std::uint8_t> Decoder::flush() {
    std::vector<std::uint8_t> output;
    impl_->flush(output);
    return output;
}

void Decoder::flush(std::vector<std::uint8_t> &output) { impl_->flush(output); }

} // namespace airspy_tv::dvbt
