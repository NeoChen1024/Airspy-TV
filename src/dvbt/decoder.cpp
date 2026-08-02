#include "airspy_tv/dvbt/decoder.hpp"

#include <complex>
#include <cstddef>
#include <memory>
#include <span>
#include <stdexcept>
#include <vector>

namespace airspy_tv::dvbt {

struct Decoder::Impl {
    explicit Impl(const DecoderParameters selected_parameters)
        : parameters(selected_parameters), demapper(parameters.constellation),
          symbol_deinterleaver(parameters.mode),
          transport_decoder(parameters.code_rate) {
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
        demapper.demap(equalized_carriers, reliability, demapped);
        symbol_deinterleaver.process(demapped, bits, symbol_index,
                                     symbol_metrics);
        bit_deinterleave(symbol_metrics, bits, bit_metrics);
        return transport_decoder.process(bit_metrics);
    }

    DecoderParameters parameters;
    MaxLogDemapper demapper;
    SymbolDeinterleaver symbol_deinterleaver;
    TransportDecoder transport_decoder;
    std::vector<float> demapped;
    std::vector<float> symbol_metrics;
    std::vector<float> bit_metrics;
};

Decoder::Decoder(const DecoderParameters parameters)
    : impl_(std::make_unique<Impl>(parameters)) {}

Decoder::~Decoder() noexcept = default;
Decoder::Decoder(Decoder &&) noexcept = default;
Decoder &Decoder::operator=(Decoder &&) noexcept = default;

void Decoder::reset() { impl_->transport_decoder.reset(); }

std::vector<std::uint8_t> Decoder::process_symbol(
    const std::span<const std::complex<float>> equalized_carriers,
    const std::span<const float> reliability, const std::size_t symbol_index) {
    return impl_->process_symbol(equalized_carriers, reliability, symbol_index);
}

DecoderParameters Decoder::parameters() const noexcept {
    return impl_->parameters;
}

TransportDecoderStats Decoder::stats() const {
    return impl_->transport_decoder.stats();
}

} // namespace airspy_tv::dvbt
