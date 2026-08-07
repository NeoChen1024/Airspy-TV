#include "airspy_tv/dvbt/transport_decoder.hpp"

#include "airspy_tv/fec/outer_fec.hpp"
#include "airspy_tv/fec/soft_viterbi.hpp"

#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace airspy_tv::dvbt {

struct TransportDecoder::Impl {
    Impl(const CodeRate selected_code_rate, const std::size_t viterbi_workers)
        : code_rate(selected_code_rate), viterbi(viterbi_workers) {
        reset();
    }

    void reset() {
        punctured_metrics.clear();
        viterbi.reset();
        outer.reset();
        statistics = {};
        statistics.viterbi_workers = viterbi.worker_count();
    }

    [[nodiscard]] std::vector<std::uint8_t>
    process(const std::span<const float> input) {
        punctured_metrics.insert(punctured_metrics.end(), input.begin(),
                                 input.end());
        const std::size_t period = [&] {
            switch (code_rate) {
            case CodeRate::rate_1_2:
                return std::size_t{2};
            case CodeRate::rate_2_3:
                return std::size_t{3};
            case CodeRate::rate_3_4:
                return std::size_t{4};
            case CodeRate::rate_5_6:
                return std::size_t{6};
            case CodeRate::rate_7_8:
                return std::size_t{8};
            }
            return std::size_t{0};
        }();
        const std::size_t aligned =
            punctured_metrics.size() - (punctured_metrics.size() % period);
        if (aligned == 0) {
            return {};
        }

        std::vector<float> mother(depunctured_size(aligned, code_rate));
        depuncture(std::span<const float>{punctured_metrics}.first(aligned),
                   code_rate, mother);
        punctured_metrics.erase(punctured_metrics.begin(),
                                punctured_metrics.begin() +
                                    static_cast<std::ptrdiff_t>(aligned));

        auto decoded = viterbi.process(mother);
        return process_viterbi_output(decoded);
    }

    [[nodiscard]] std::vector<std::uint8_t>
    process_soft(const std::span<const std::uint8_t> mother_metrics) {
        return process_viterbi_output(viterbi.process_soft(mother_metrics));
    }

    [[nodiscard]] std::vector<std::uint8_t> flush() {
        return process_viterbi_output(viterbi.flush());
    }

    [[nodiscard]] std::vector<std::uint8_t>
    process_viterbi_output(const std::span<const std::uint8_t> decoded) {
        constexpr std::size_t output_bytes = fec::viterbi_output_bytes;
        if (decoded.size() % output_bytes != 0) {
            throw std::runtime_error("misaligned Viterbi window output");
        }
        std::vector<std::uint8_t> transport_stream;
        for (std::size_t offset = 0; offset < decoded.size();
             offset += output_bytes) {
            statistics.viterbi_bits += output_bytes * 8;
            auto packets = outer.process(decoded.subspan(offset, output_bytes));
            transport_stream.insert(transport_stream.end(), packets.begin(),
                                    packets.end());
        }
        return transport_stream;
    }

    CodeRate code_rate;
    fec::SoftViterbi viterbi;
    fec::OuterFec outer;
    std::vector<float> punctured_metrics;
    TransportDecoderStats statistics;
};

TransportDecoder::TransportDecoder(const CodeRate code_rate,
                                   const std::size_t viterbi_workers)
    : impl_(std::make_unique<Impl>(code_rate, viterbi_workers)) {}

TransportDecoder::~TransportDecoder() noexcept = default;
TransportDecoder::TransportDecoder(TransportDecoder &&) noexcept = default;
TransportDecoder &
TransportDecoder::operator=(TransportDecoder &&) noexcept = default;

void TransportDecoder::reset() { impl_->reset(); }

void TransportDecoder::set_diagnostic_handler(DiagnosticEventHandler handler) {
    impl_->outer.set_diagnostic_handler(std::move(handler));
}

std::vector<std::uint8_t>
TransportDecoder::process(const std::span<const float> punctured_llrs) {
    return impl_->process(punctured_llrs);
}

std::vector<std::uint8_t> TransportDecoder::process_soft(
    const std::span<const std::uint8_t> mother_metrics) {
    return impl_->process_soft(mother_metrics);
}

std::vector<std::uint8_t> TransportDecoder::flush() { return impl_->flush(); }

TransportDecoderStats TransportDecoder::stats() const {
    auto statistics = impl_->statistics;
    const auto [errors, compared] = impl_->viterbi.error_counts();
    statistics.pre_viterbi_error_bits = errors;
    statistics.pre_viterbi_compared_bits = compared;
    const auto outer_stats = impl_->outer.stats();
    statistics.post_viterbi_error_bits = outer_stats.corrected_payload_bits;
    statistics.post_viterbi_compared_bits = outer_stats.compared_payload_bits;
    statistics.rs_packets = outer_stats.rs_packets;
    statistics.rs_uncorrectable_packets = outer_stats.rs_uncorrectable_packets;
    statistics.tei_packets = outer_stats.tei_packets;
    statistics.ts_packets = outer_stats.ts_packets;
    statistics.outer_deinterleaver_phase =
        outer_stats.outer_deinterleaver_phase;
    statistics.outer_sync_distance = outer_stats.outer_sync_distance;
    statistics.outer_rs_evidence = outer_stats.outer_rs_evidence;
    statistics.rs_synchronized = outer_stats.rs_synchronized;
    statistics.energy_synchronized = outer_stats.energy_synchronized;
    return statistics;
}

} // namespace airspy_tv::dvbt
