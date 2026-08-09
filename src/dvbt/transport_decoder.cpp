#include "airspy_tv/dvbt/transport_decoder.hpp"

#include "airspy_tv/fec/outer_fec.hpp"
#include "airspy_tv/fec/soft_viterbi.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

namespace airspy_tv::dvbt {
namespace {

class TimingScope {
  public:
    explicit TimingScope(double *destination) noexcept
        : destination_(destination),
          started_at_(destination == nullptr
                          ? std::chrono::steady_clock::time_point{}
                          : std::chrono::steady_clock::now()) {}

    ~TimingScope() {
        if (destination_ != nullptr) {
            *destination_ += std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - started_at_)
                                 .count();
        }
    }

  private:
    double *destination_;
    std::chrono::steady_clock::time_point started_at_;
};

} // namespace

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
        timing = {};
    }

    void set_detailed_timing_enabled(const bool enabled) noexcept {
        detailed_timing_enabled = enabled;
        viterbi.set_detailed_timing_enabled(enabled);
        outer.set_detailed_timing_enabled(enabled);
    }

    [[nodiscard]] double *timer(double &destination) noexcept {
        return detailed_timing_enabled ? &destination : nullptr;
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

        std::vector<std::uint8_t> decoded;
        {
            TimingScope viterbi_timer(timer(timing.viterbi_wall_ms));
            decoded = viterbi.process(mother);
        }
        return process_viterbi_output(decoded);
    }

    [[nodiscard]] std::vector<std::uint8_t>
    process_soft(const std::span<const std::uint8_t> mother_metrics) {
        std::vector<std::uint8_t> decoded;
        {
            TimingScope viterbi_timer(timer(timing.viterbi_wall_ms));
            decoded = viterbi.process_soft(mother_metrics);
        }
        return process_viterbi_output(decoded);
    }

    [[nodiscard]] std::vector<std::uint8_t> flush() {
        std::vector<std::uint8_t> decoded;
        {
            TimingScope viterbi_timer(timer(timing.viterbi_wall_ms));
            decoded = viterbi.flush();
        }
        return process_viterbi_output(decoded);
    }

    [[nodiscard]] std::vector<std::uint8_t>
    process_viterbi_output(const std::span<const std::uint8_t> decoded) {
        constexpr std::size_t output_bytes = fec::viterbi_output_bytes;
        if (decoded.size() % output_bytes != 0) {
            throw std::runtime_error("misaligned Viterbi window output");
        }
        const auto handoff_started_at =
            detailed_timing_enabled ? std::chrono::steady_clock::now()
                                    : std::chrono::steady_clock::time_point{};
        const double outer_started_ms = timing.outer_wall_ms;
        const double output_started_ms = timing.transport_output_ms;
        std::vector<std::uint8_t> transport_stream;
        for (std::size_t offset = 0; offset < decoded.size();
             offset += output_bytes) {
            statistics.viterbi_bits += output_bytes * 8;
            std::vector<std::uint8_t> packets;
            {
                TimingScope outer_timer(timer(timing.outer_wall_ms));
                packets = outer.process(decoded.subspan(offset, output_bytes));
            }
            {
                TimingScope output_timer(timer(timing.transport_output_ms));
                transport_stream.insert(transport_stream.end(), packets.begin(),
                                        packets.end());
            }
        }
        if (detailed_timing_enabled) {
            const double elapsed =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - handoff_started_at)
                    .count();
            timing.decoded_handoff_ms += std::max(
                0.0, elapsed - (timing.outer_wall_ms - outer_started_ms) -
                         (timing.transport_output_ms - output_started_ms));
        }
        return transport_stream;
    }

    [[nodiscard]] TransportDecoderTiming current_timing() const noexcept {
        auto result = timing;
        const auto viterbi_timing = viterbi.timing();
        result.viterbi_submit_ms = viterbi_timing.submit_ms;
        result.viterbi_queue_wait_ms = viterbi_timing.queue_wait_ms;
        result.viterbi_collect_ms = viterbi_timing.collect_ms;
        result.viterbi_flush_wait_ms = viterbi_timing.flush_wait_ms;
        result.viterbi_worker_work_ms = viterbi_timing.aggregate_worker_work_ms;
        const auto outer_timing = outer.timing();
        result.outer_alignment_ms = outer_timing.alignment_ms;
        result.outer_bit_repack_ms = outer_timing.bit_repack_ms;
        result.outer_byte_deinterleave_ms = outer_timing.byte_deinterleave_ms;
        result.outer_rs_decode_ms = outer_timing.rs_decode_ms;
        result.outer_rs_codeword_copy_ms =
            outer_timing.rs_codeword_copy_ms;
        result.outer_rs_syndrome_ms = outer_timing.rs_syndrome_ms;
        result.outer_rs_error_locator_ms = outer_timing.rs_error_locator_ms;
        result.outer_rs_correction_ms = outer_timing.rs_correction_ms;
        result.outer_rs_payload_copy_ms = outer_timing.rs_payload_copy_ms;
        result.outer_energy_tei_ms = outer_timing.energy_tei_ms;
        result.outer_buffer_ms = outer_timing.buffer_ms;
        result.outer_output_ms = outer_timing.output_ms;
        return result;
    }

    CodeRate code_rate;
    fec::SoftViterbi viterbi;
    fec::OuterFec outer;
    std::vector<float> punctured_metrics;
    TransportDecoderStats statistics;
    bool detailed_timing_enabled{};
    TransportDecoderTiming timing;
};

TransportDecoder::TransportDecoder(const CodeRate code_rate,
                                   const std::size_t viterbi_workers)
    : impl_(std::make_unique<Impl>(code_rate, viterbi_workers)) {}

TransportDecoder::~TransportDecoder() noexcept = default;
TransportDecoder::TransportDecoder(TransportDecoder &&) noexcept = default;
TransportDecoder &
TransportDecoder::operator=(TransportDecoder &&) noexcept = default;

void TransportDecoder::reset() { impl_->reset(); }

void TransportDecoder::set_detailed_timing_enabled(
    const bool enabled) noexcept {
    impl_->set_detailed_timing_enabled(enabled);
}

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
    statistics.rs_clean_packets = outer_stats.rs_clean_packets;
    statistics.rs_corrected_packets = outer_stats.rs_corrected_packets;
    statistics.rs_uncorrectable_packets = outer_stats.rs_uncorrectable_packets;
    statistics.tei_packets = outer_stats.tei_packets;
    statistics.ts_packets = outer_stats.ts_packets;
    statistics.outer_bit_offset = outer_stats.outer_bit_offset;
    statistics.outer_deinterleaver_phase =
        outer_stats.outer_deinterleaver_phase;
    statistics.outer_sync_distance = outer_stats.outer_sync_distance;
    statistics.outer_rs_evidence = outer_stats.outer_rs_evidence;
    statistics.rs_synchronized = outer_stats.rs_synchronized;
    statistics.energy_synchronized = outer_stats.energy_synchronized;
    return statistics;
}

TransportDecoderTiming TransportDecoder::timing() const noexcept {
    return impl_->current_timing();
}

} // namespace airspy_tv::dvbt
