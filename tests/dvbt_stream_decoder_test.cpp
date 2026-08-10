#include "airspy_tv/dvbt/stream_decoder.hpp"

#include "airspy_tv/dvbt/inner_decoder.hpp"
#include "airspy_tv/dvbt/ofdm_acquisition.hpp"
#include "airspy_tv/dvbt/soft_demapper.hpp"
#include "airspy_tv/fec/reed_solomon.hpp"
#include <fftw3.h>

#include <algorithm>
#include <array>
#include <barrier>
#include <bit>
#include <chrono>
#include <cmath>
#include <complex>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <mutex>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace {

using airspy_tv::TransportDiscontinuity;
using airspy_tv::dvbt::CodeRate;
using airspy_tv::dvbt::Constellation;
using airspy_tv::dvbt::GuardInterval;
using airspy_tv::dvbt::MaxLogDemapper;
using airspy_tv::dvbt::SignalAnalysisSnapshot;
using airspy_tv::dvbt::SignalAnalysisSource;
using airspy_tv::dvbt::StreamDecoder;
using airspy_tv::dvbt::SymbolDeinterleaver;
using airspy_tv::dvbt::TransmissionMode;
using airspy_tv::fec::DvbReedSolomon;

constexpr std::size_t ts_packet_size = 188;
constexpr std::size_t rs_packet_size = 204;
constexpr std::size_t fft_size = 8192;
constexpr std::size_t guard_size = 2048;
constexpr std::size_t symbol_size = fft_size + guard_size;
constexpr std::size_t maximum_carrier = 6816;
constexpr std::size_t payload_carriers = 6048;
constexpr std::size_t bits_per_carrier = 2;
constexpr std::size_t symbols = 68;
constexpr std::size_t packet_count = 248;
constexpr std::uint32_t sample_rate = 8'000'000;
constexpr std::uint32_t channel_bandwidth = 7'000'000;

constexpr std::array continual_2k{
    0,    48,   54,   87,   141,  156,  192,  201,  255,  279,  282,  333,
    432,  450,  483,  525,  531,  618,  636,  714,  759,  765,  780,  804,
    873,  888,  918,  939,  942,  969,  984,  1050, 1101, 1107, 1110, 1137,
    1140, 1146, 1206, 1269, 1323, 1377, 1491, 1683, 1704};
constexpr std::array tps_2k{34,  50,   209,  346,  413,  569,  595,  688, 790,
                            901, 1073, 1219, 1262, 1286, 1469, 1594, 1687};

[[nodiscard]] bool listed(const std::span<const int> list,
                          const std::size_t value) {
    return std::binary_search(list.begin(), list.end(),
                              static_cast<int>(value));
}

[[nodiscard]] constexpr std::array<std::uint8_t, 6817> make_prbs() {
    std::array<std::uint8_t, 6817> result{};
    std::uint32_t state = 0x7ffU;
    for (auto &bit : result) {
        bit = static_cast<std::uint8_t>(state & 1U);
        state = (state >> 1U) | ((((state >> 2U) ^ state) & 1U) << 10U);
    }
    return result;
}
constexpr auto prbs = make_prbs();

void require(const bool condition, const std::string_view message) {
    if (!condition) {
        throw std::runtime_error(std::string{message});
    }
}

[[nodiscard]] std::uint8_t clock_prbs(std::uint16_t &shift_register) {
    std::uint8_t result = 0;
    for (int bit = 0; bit < 8; ++bit) {
        const std::uint16_t feedback =
            ((shift_register >> 13U) ^ (shift_register >> 14U)) & 1U;
        shift_register = static_cast<std::uint16_t>(
            ((static_cast<unsigned int>(shift_register) << 1U) | feedback) &
            0x7FFFU);
        result = static_cast<std::uint8_t>((result << 1U) | feedback);
    }
    return result;
}

[[nodiscard]] std::vector<std::uint8_t>
make_transport_stream(const std::size_t count, const std::uint8_t salt = 0) {
    std::vector<std::uint8_t> stream(count * ts_packet_size);
    for (std::size_t packet = 0; packet < count; ++packet) {
        const std::size_t offset = packet * ts_packet_size;
        stream[offset] = 0x47;
        stream[offset + 1] = 0x1F;
        stream[offset + 2] = 0xFF;
        stream[offset + 3] = static_cast<std::uint8_t>(
            0x10U | static_cast<std::uint8_t>(packet & 0x0FU));
        for (std::size_t byte = 4; byte < ts_packet_size; ++byte) {
            stream[offset + byte] = static_cast<std::uint8_t>(
                ((packet * 29U) + (byte * 17U) + 3U + salt) & 0xFFU);
        }
    }
    return stream;
}

[[nodiscard]] std::vector<std::uint8_t>
energy_scramble(const std::span<const std::uint8_t> transport_stream) {
    require(transport_stream.size() % (8 * ts_packet_size) == 0,
            "TS stream must contain complete energy frames");
    std::vector<std::uint8_t> output(transport_stream.size());
    for (std::size_t group = 0; group < transport_stream.size();
         group += 8 * ts_packet_size) {
        std::uint16_t shift_register = 0x00A9;
        for (std::size_t packet = 0; packet < 8; ++packet) {
            const std::size_t offset = group + (packet * ts_packet_size);
            output[offset] = packet == 0 ? 0xB8 : 0x47;
            for (std::size_t byte = 1; byte < ts_packet_size; ++byte) {
                output[offset + byte] = transport_stream[offset + byte] ^
                                        clock_prbs(shift_register);
            }
            static_cast<void>(clock_prbs(shift_register));
        }
    }
    return output;
}

[[nodiscard]] std::vector<std::uint8_t>
rs_encode(const std::span<const std::uint8_t> randomized) {
    DvbReedSolomon codec;
    std::vector<std::uint8_t> output((randomized.size() / ts_packet_size) *
                                     rs_packet_size);
    for (std::size_t input = 0, encoded = 0; input < randomized.size();
         input += ts_packet_size, encoded += rs_packet_size) {
        codec.encode(randomized.subspan(input, ts_packet_size),
                     std::span{output}.subspan(encoded, rs_packet_size));
    }
    return output;
}

[[nodiscard]] std::vector<std::uint8_t>
outer_interleave(const std::span<const std::uint8_t> input) {
    constexpr std::size_t branches = 12;
    constexpr std::size_t step = 17;
    std::array<std::vector<std::uint8_t>, branches> queues;
    for (std::size_t branch = 0; branch < branches; ++branch) {
        queues[branch].assign(branch * step, 0);
    }
    std::vector<std::uint8_t> output;
    output.reserve(input.size());
    std::size_t branch = 0;
    for (const std::uint8_t value : input) {
        auto &queue = queues[branch];
        queue.push_back(value);
        output.push_back(queue.front());
        queue.erase(queue.begin());
        branch = (branch + 1) % branches;
    }
    return output;
}

[[nodiscard]] std::vector<std::uint8_t>
convolutional_encode(const std::span<const std::uint8_t> input) {
    constexpr std::array<std::uint8_t, 2> polynomials{0117, 0155};
    std::vector<std::uint8_t> output;
    output.reserve(input.size() * 16);
    std::uint8_t shift_register = 0;
    for (const std::uint8_t value : input) {
        for (int bit = 7; bit >= 0; --bit) {
            shift_register = static_cast<std::uint8_t>(
                ((static_cast<unsigned int>(shift_register) << 1U) |
                 ((value >> static_cast<unsigned int>(bit)) & 1U)) &
                0x7FU);
            for (const std::uint8_t polynomial : polynomials) {
                output.push_back(static_cast<std::uint8_t>(
                    static_cast<unsigned int>(
                        std::popcount(static_cast<unsigned int>(shift_register &
                                                                polynomial))) &
                    1U));
            }
        }
    }
    return output;
}

[[nodiscard]] std::vector<float>
make_metrics(const std::span<const std::uint8_t> bits) {
    std::vector<float> metrics;
    metrics.reserve(bits.size());
    for (const std::uint8_t bit : bits) {
        metrics.push_back(bit == 0U ? -16.0F : 16.0F);
    }
    return metrics;
}

[[nodiscard]] std::vector<float>
bit_interleave(const std::span<const float> input) {
    constexpr std::array<std::size_t, 6> offsets{0, 63, 105, 42, 21, 84};
    constexpr std::size_t block_carriers = 126;
    const std::size_t block_metrics = block_carriers * bits_per_carrier;
    require(input.size() % block_metrics == 0, "bit interleaver alignment");
    std::vector<float> output(input.size());
    for (std::size_t block = 0; block < input.size(); block += block_metrics) {
        for (std::size_t position = 0; position < block_carriers; ++position) {
            for (std::size_t output_bit = 0; output_bit < bits_per_carrier;
                 ++output_bit) {
                const std::size_t half = bits_per_carrier / 2;
                const std::size_t source_bit =
                    (output_bit / half) + (2 * (output_bit % half));
                const std::size_t source_position =
                    (position + block_carriers - offsets[source_bit]) %
                    block_carriers;
                output[block + (source_position * bits_per_carrier) +
                       source_bit] =
                    input[block + (position * bits_per_carrier) + output_bit];
            }
        }
    }
    return output;
}

[[nodiscard]] std::vector<float>
symbol_interleave(const std::span<const float> input,
                  const SymbolDeinterleaver &deinterleaver,
                  const std::size_t symbol_index) {
    const auto permutation = deinterleaver.permutation();
    std::vector<float> output(input.size());
    for (std::size_t carrier = 0; carrier < permutation.size(); ++carrier) {
        const std::size_t input_carrier =
            (symbol_index & 1U) != 0U ? carrier : permutation[carrier];
        const std::size_t output_carrier =
            (symbol_index & 1U) != 0U ? permutation[carrier] : carrier;
        std::copy(input.begin() + static_cast<std::ptrdiff_t>(output_carrier *
                                                              bits_per_carrier),
                  input.begin() + static_cast<std::ptrdiff_t>(
                                      (output_carrier + 1) * bits_per_carrier),
                  output.begin() + static_cast<std::ptrdiff_t>(
                                       input_carrier * bits_per_carrier));
    }
    return output;
}

struct EncodedStream {
    std::vector<std::uint8_t> transport;
    std::vector<float> metrics;
};

[[nodiscard]] EncodedStream encode_transport(const std::uint8_t salt = 0) {
    EncodedStream result;
    result.transport = make_transport_stream(packet_count, salt);
    const auto randomized = energy_scramble(result.transport);
    const auto rs = rs_encode(randomized);
    const auto interleaved = outer_interleave(rs);
    const auto encoded = convolutional_encode(interleaved);
    result.metrics = make_metrics(encoded);
    const std::size_t target_metrics =
        symbols * payload_carriers * bits_per_carrier;
    require(result.metrics.size() <= target_metrics,
            "8K test stream exceeds the synthetic symbol budget");
    // 248 packets is an energy-frame aligned prefix of the 252 packets needed
    // to fill exactly 68 QPSK symbols at rate 1/2. Neutral padding supplies
    // the final partial FEC codewords; the assertion below checks a complete
    // known TS run and ignores only the synthetic tail/warmup.
    result.metrics.resize(target_metrics, -16.0F);
    return result;
}

[[nodiscard]] std::vector<std::size_t>
payload_indices(const std::size_t phase) {
    std::vector<std::size_t> result;
    result.reserve(payload_carriers);
    for (std::size_t carrier = 0; carrier <= maximum_carrier; ++carrier) {
        const std::size_t base = carrier % 1704;
        const bool continual = listed(continual_2k, base);
        const bool scattered = carrier % 12 == phase * 3;
        const bool tps = listed(tps_2k, base);
        if (!continual && !scattered && !tps) {
            result.push_back(carrier);
        }
    }
    require(result.size() == payload_carriers,
            "8K payload carrier count mismatch");
    return result;
}

[[nodiscard]] std::complex<float> pilot_value(const std::size_t carrier) {
    return prbs[carrier] == 0U ? std::complex<float>{4.0F / 3.0F, 0.0F}
                               : std::complex<float>{-4.0F / 3.0F, 0.0F};
}

[[nodiscard]] constexpr std::array<std::uint8_t, 68> make_tps_frame() {
    std::array<std::uint8_t, 68> bits{};
    constexpr std::array<std::uint8_t, 16> sync_even{
        0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0};
    for (std::size_t index = 0; index < sync_even.size(); ++index) {
        bits[1 + index] = sync_even[index];
    }
    // QPSK, non-hierarchical, HP/LP 1/2, guard 1/4, 8K. Frame number,
    // cell ID, and reserved fields stay zero.
    bits[36] = 1;
    bits[37] = 1;
    bits[39] = 1;

    unsigned int reg = 0;
    for (std::size_t input = 0; input < 113; ++input) {
        const unsigned int data = input < 60 ? 0U : bits[1 + input - 60];
        const unsigned int feedback = (data ^ reg) & 1U;
        reg >>= 1U;
        reg |= feedback << 13U;
        reg ^= (feedback << 12U) ^ (feedback << 11U) ^
               (feedback << 9U) ^ (feedback << 8U) ^
               (feedback << 7U) ^ (feedback << 5U) ^ (feedback << 4U);
    }
    for (std::size_t bit = 0; bit < 14; ++bit) {
        bits[54 + bit] = static_cast<std::uint8_t>((reg >> bit) & 1U);
    }
    return bits;
}

constexpr auto tps_frame = make_tps_frame();

[[nodiscard]] constexpr float tps_sign(const std::size_t symbol) {
    float sign = 1.0F;
    for (std::size_t index = 0; index < symbol; ++index) {
        if (tps_frame[index % tps_frame.size()] != 0U) {
            sign = -sign;
        }
    }
    return sign;
}

[[nodiscard]] std::vector<std::int16_t>
make_iq(const std::span<const float> metrics, const float payload_gain = 1.0F,
        const bool valid_tps = false,
        const std::size_t total_symbols = symbols,
        const std::size_t tps_start_symbol = 0) {
    const MaxLogDemapper demapper{Constellation::qpsk};
    const SymbolDeinterleaver symbol_permutation{TransmissionMode::k8};
    const auto points = demapper.constellation_points();
    auto *frequency = static_cast<fftwf_complex *>(
        fftwf_malloc(sizeof(fftwf_complex) * fft_size));
    auto *time = static_cast<fftwf_complex *>(
        fftwf_malloc(sizeof(fftwf_complex) * fft_size));
    require(frequency != nullptr && time != nullptr, "allocate OFDM FFT");
    fftwf_plan plan = fftwf_plan_dft_1d(static_cast<int>(fft_size), frequency,
                                        time, FFTW_BACKWARD, FFTW_ESTIMATE);
    require(plan != nullptr, "create OFDM IFFT");

    std::vector<std::int16_t> result;
    result.reserve(total_symbols * symbol_size * 2);
    for (std::size_t symbol = 0; symbol < total_symbols; ++symbol) {
        std::fill(reinterpret_cast<float *>(frequency),
                  reinterpret_cast<float *>(frequency) + (2 * fft_size), 0.0F);
        const std::size_t phase = symbol % 4;
        const auto payload = payload_indices(phase);
        const auto symbol_metrics =
            metrics.subspan((symbol % symbols) * payload_carriers *
                                bits_per_carrier,
                            payload_carriers * bits_per_carrier);
        const auto bit_metrics = bit_interleave(symbol_metrics);
        const auto transmitted =
            symbol_interleave(bit_metrics, symbol_permutation, symbol);

        std::size_t payload_position = 0;
        for (std::size_t carrier = 0; carrier <= maximum_carrier; ++carrier) {
            const std::size_t base = carrier % 1704;
            const bool continual = listed(continual_2k, base);
            const bool scattered = carrier % 12 == phase * 3;
            const bool tps = listed(tps_2k, base);
            std::complex<float> value{};
            if (continual || scattered) {
                value = pilot_value(carrier);
            } else if (tps && valid_tps && symbol >= tps_start_symbol) {
                value = pilot_value(carrier) *
                        tps_sign(symbol - tps_start_symbol);
            } else if (!tps) {
                std::size_t label = 0;
                for (std::size_t bit = 0; bit < bits_per_carrier; ++bit) {
                    label = (label << 1U) |
                            (transmitted[(payload_position * bits_per_carrier) +
                                         bit] > 0.0F
                                 ? 1U
                                 : 0U);
                }
                value = points[label] * payload_gain;
                ++payload_position;
            }
            const auto bin = static_cast<std::ptrdiff_t>(carrier) -
                             static_cast<std::ptrdiff_t>(maximum_carrier / 2);
            const auto wrapped = (bin + static_cast<std::ptrdiff_t>(fft_size)) %
                                 static_cast<std::ptrdiff_t>(fft_size);
            frequency[wrapped][0] = value.real();
            frequency[wrapped][1] = value.imag();
        }
        require(payload_position == payload_carriers,
                "8K payload mapping count mismatch");
        fftwf_execute(plan);
        for (std::size_t index = 0; index < guard_size; ++index) {
            const std::size_t source = fft_size - guard_size + index;
            float real = time[source][0] / static_cast<float>(fft_size);
            float imag = time[source][1] / static_cast<float>(fft_size);
            // Keep the first symbol as the deterministic acquisition anchor.
            // A small, CP-only impairment on later prefixes makes the
            // acquisition score prefer that anchor instead of choosing an
            // arbitrary equal-score symbol at the end of an ideal capture.
            if (symbol != 0 &&
                (!valid_tps || symbol < tps_start_symbol)) {
                const float noise =
                    0.002F * std::sin((static_cast<float>(index) * 0.071F) +
                                      static_cast<float>(symbol));
                real += noise;
                imag += 0.8F * noise;
            }
            result.push_back(
                static_cast<std::int16_t>(std::lround(real * 32767.0F)));
            result.push_back(
                static_cast<std::int16_t>(std::lround(imag * 32767.0F)));
        }
        for (std::size_t index = 0; index < fft_size; ++index) {
            const float real = time[index][0] / static_cast<float>(fft_size);
            const float imag = time[index][1] / static_cast<float>(fft_size);
            result.push_back(
                static_cast<std::int16_t>(std::lround(real * 32767.0F)));
            result.push_back(
                static_cast<std::int16_t>(std::lround(imag * 32767.0F)));
        }
    }
    fftwf_destroy_plan(plan);
    fftwf_free(time);
    fftwf_free(frequency);
    return result;
}

[[nodiscard]] std::vector<std::int16_t>
apply_cfo_drift(const std::span<const std::int16_t> iq,
                const double initial_cfo_hz,
                const double cfo_drift_hz_per_second) {
    require(iq.size() % 2U == 0U, "CFO input must contain complete I/Q pairs");
    std::vector<std::int16_t> result(iq.size());
    const auto sample_rate_hz = static_cast<double>(sample_rate);
    const double phase_step =
        2.0 * std::numbers::pi *
        ((initial_cfo_hz / sample_rate_hz) +
         (0.5 * cfo_drift_hz_per_second / (sample_rate_hz * sample_rate_hz)));
    const double step_acceleration = 2.0 * std::numbers::pi *
                                     cfo_drift_hz_per_second /
                                     (sample_rate_hz * sample_rate_hz);
    std::complex<double> oscillator{1.0, 0.0};
    std::complex<double> oscillator_step = std::polar(1.0, phase_step);
    const std::complex<double> oscillator_acceleration =
        std::polar(1.0, step_acceleration);
    for (std::size_t index = 0; index < iq.size() / 2U; ++index) {
        const std::complex<double> input{
            static_cast<double>(iq[2U * index]),
            static_cast<double>(iq[(2U * index) + 1U])};
        const auto shifted = input * oscillator;
        const auto quantize = [](const double value) {
            return static_cast<std::int16_t>(
                std::clamp(std::lround(value), -32768L, 32767L));
        };
        result[2U * index] = quantize(shifted.real());
        result[(2U * index) + 1U] = quantize(shifted.imag());
        oscillator *= oscillator_step;
        oscillator_step *= oscillator_acceleration;
        if ((index & 4095U) == 4095U) {
            const auto next_index = static_cast<double>(index + 1U);
            const double next_time = next_index / sample_rate_hz;
            oscillator = std::polar(1.0, 2.0 * std::numbers::pi *
                                             ((initial_cfo_hz * next_time) +
                                              (0.5 * cfo_drift_hz_per_second *
                                               next_time * next_time)));
            oscillator_step = std::polar(
                1.0, 2.0 * std::numbers::pi *
                         ((initial_cfo_hz / sample_rate_hz) +
                          (cfo_drift_hz_per_second * (next_index + 0.5) /
                           (sample_rate_hz * sample_rate_hz))));
        }
    }
    return result;
}

[[nodiscard]] std::vector<std::int16_t>
apply_cfo(const std::span<const std::int16_t> iq, const double cfo_hz) {
    return apply_cfo_drift(iq, cfo_hz, 0.0);
}

[[nodiscard]] std::vector<std::int16_t>
apply_cfo_step(const std::span<const std::int16_t> iq,
               const std::size_t step_sample, const double initial_cfo_hz,
               const double final_cfo_hz) {
    std::vector<std::int16_t> result(iq.size());
    double phase = 0.0;
    for (std::size_t sample = 0; sample < iq.size() / 2U; ++sample) {
        const double cfo_hz =
            sample < step_sample ? initial_cfo_hz : final_cfo_hz;
        const std::complex<double> oscillator = std::polar(1.0, phase);
        const std::complex<double> value{
            static_cast<double>(iq[sample * 2U]),
            static_cast<double>(iq[(sample * 2U) + 1U])};
        const auto shifted = value * oscillator;
        result[sample * 2U] = static_cast<std::int16_t>(std::clamp(
            std::lround(shifted.real()),
            static_cast<long>(std::numeric_limits<std::int16_t>::min()),
            static_cast<long>(std::numeric_limits<std::int16_t>::max())));
        result[(sample * 2U) + 1U] = static_cast<std::int16_t>(std::clamp(
            std::lround(shifted.imag()),
            static_cast<long>(std::numeric_limits<std::int16_t>::min()),
            static_cast<long>(std::numeric_limits<std::int16_t>::max())));
        phase = std::remainder(phase + (2.0 * std::numbers::pi * cfo_hz /
                                        static_cast<double>(sample_rate)),
                               2.0 * std::numbers::pi);
    }
    return result;
}

[[nodiscard]] std::vector<std::int16_t>
repeat_iq(const std::span<const std::int16_t> iq, const std::size_t count) {
    std::vector<std::int16_t> result;
    result.reserve(iq.size() * count);
    for (std::size_t copy = 0; copy < count; ++copy) {
        result.insert(result.end(), iq.begin(), iq.end());
    }
    return result;
}

struct DecodeResult {
    std::vector<std::uint8_t> transport;
    std::vector<TransportDiscontinuity> discontinuities;
    airspy_tv::dvbt::StreamDecoderStats stats;
    SignalAnalysisSnapshot analysis;
    airspy_tv::SignalSnapshot signal;
    airspy_tv::PipelineSnapshot pipeline;
};

void submit_in_blocks(StreamDecoder &decoder,
                      const std::span<const std::int16_t> iq,
                      const std::span<const std::size_t> block_sizes) {
    std::size_t offset = 0;
    std::size_t block = 0;
    while (offset < iq.size()) {
        const std::size_t requested = block_sizes[block % block_sizes.size()];
        const std::size_t complex_count =
            std::min(requested, (iq.size() - offset) / 2);
        if (complex_count == 0) {
            break; // only a stray scalar remains; do not spin on an empty
                   // submit
        }
        decoder.submit_blocking(iq.subspan(offset, complex_count * 2),
                                sample_rate, channel_bandwidth);
        offset += complex_count * 2;
        ++block;
    }
}

[[nodiscard]] bool has_known_run(const std::span<const std::uint8_t> output,
                                 const std::span<const std::uint8_t> expected) {
    constexpr std::size_t trellis_warmup_packets = 8;
    constexpr std::size_t probe_packets = 32;
    const auto probe_begin =
        expected.begin() + trellis_warmup_packets * ts_packet_size;
    const auto probe_end = probe_begin + probe_packets * ts_packet_size;
    return std::search(output.begin(), output.end(), probe_begin, probe_end) !=
           output.end();
}

[[nodiscard]] DecodeResult
decode_in_blocks(const std::span<const std::int16_t> iq,
                 const std::span<const std::size_t> block_sizes,
                 const std::size_t worker_threads = 8,
                 const std::size_t queue_capacity_multiplier = 1) {
    DecodeResult result;
    std::mutex callback_mutex;
    StreamDecoder decoder;
    decoder.set_parameters(
        {.channel_bandwidth_hz = channel_bandwidth,
         .mode = TransmissionMode::k8,
         .guard_interval = GuardInterval::gi_1_4,
         .constellation = Constellation::qpsk,
         .code_rate = CodeRate::rate_1_2,
         .worker_threads = worker_threads,
         .queue_capacity_multiplier = queue_capacity_multiplier});
    decoder.set_transport_callback(
        [&result, &callback_mutex](const std::span<const std::uint8_t> output) {
            const std::scoped_lock lock(callback_mutex);
            result.transport.insert(result.transport.end(), output.begin(),
                                    output.end());
        });
    decoder.set_discontinuity_callback(
        [&result, &callback_mutex](const TransportDiscontinuity discontinuity) {
            const std::scoped_lock lock(callback_mutex);
            result.discontinuities.push_back(discontinuity);
        });
    submit_in_blocks(decoder, iq, block_sizes);
    decoder.flush();
    decoder.wait_until_idle();
    {
        const std::scoped_lock lock(callback_mutex);
        result.stats = decoder.stats();
        result.analysis = decoder.analysis_snapshot();
        result.signal = decoder.signal_snapshot();
        result.pipeline = decoder.pipeline_snapshot();
    }
    return result;
}

void test_8k_queue_capacity_multiplier() {
    const auto encoded = encode_transport();
    const auto iq = make_iq(encoded.metrics);
    constexpr std::array<std::size_t, 5> block_sizes{4097, 12345, 8191, 777,
                                                     16384};
    const auto baseline = decode_in_blocks(iq, block_sizes, 4, 1);
    const auto scaled = decode_in_blocks(iq, block_sizes, 4, 4);
    require(scaled.stats.input_queue_capacity_samples ==
                baseline.stats.input_queue_capacity_samples,
            "offline multiplier unexpectedly changed the IQ queue");
    require(scaled.stats.symbol_queue_capacity ==
                baseline.stats.symbol_queue_capacity * 4,
            "offline multiplier did not scale the FEC queue");
    require(scaled.stats.ring_capacity_samples ==
                baseline.stats.ring_capacity_samples,
            "offline multiplier unexpectedly changed the resampled ring");
    require(scaled.transport == baseline.transport,
            "queue capacity changed decoded transport bytes");
}

void test_8k_worker_count_parity() {
    const auto encoded = encode_transport();
    const auto iq = make_iq(encoded.metrics);
    constexpr std::array<std::size_t, 8> block_sizes{17,  4097,  12345, 8191,
                                                     777, 16384, 251,   10003};
    const auto serial = decode_in_blocks(iq, block_sizes, 1);
    const auto parallel = decode_in_blocks(iq, block_sizes, 8);
    require(!serial.transport.empty(),
            "worker-parity reference produced no transport stream");
    require(serial.transport == parallel.transport,
            "symbol worker count changed ordered transport output");
}

void test_8k_clean_signal() {
    const auto encoded = encode_transport();
    const auto iq = make_iq(encoded.metrics);
    constexpr std::array<std::size_t, 8> block_sizes{17,  4097,  12345, 8191,
                                                     777, 16384, 251,   10003};
    const auto result = decode_in_blocks(iq, block_sizes);

    require(has_known_run(result.transport, encoded.transport),
            "8K synthetic I/Q did not produce a contiguous TS run");
    require(result.stats.ofdm_locked, "8K decoder did not lock OFDM");
    require(result.stats.fft_size == fft_size, "8K decoder selected wrong FFT");
    require(result.stats.guard_size == guard_size,
            "8K decoder selected wrong guard interval");
    require(result.stats.dropped_blocks == 0,
            "blocking synthetic input dropped blocks");
    require(result.stats.resample_workers == 2 &&
                result.stats.symbol_workers == 3 &&
                result.stats.transport.viterbi_workers == 3,
            "8-thread worker budget must split 2 resample + 3 symbol + 3 FEC");
    require(result.stats.timing_measurements > 0,
            "8K decoder did not publish timing measurements");
    require(result.stats.timing_measurements ==
                result.stats.timing_accepted_measurements +
                    result.stats.timing_rejected_measurements,
            "timing measurement accounting is inconsistent");
    require(result.stats.timing_confidence > 0.0F &&
                result.stats.timing_confidence <= 1.0F,
            "timing confidence is outside its normalized range");
    require(std::abs(result.stats.timing_confidence -
                     static_cast<float>(
                         result.stats.timing_accepted_measurements) /
                         static_cast<float>(result.stats.timing_measurements)) <
                1.0e-6F,
            "timing confidence does not match measurement acceptance");
    require(result.analysis.locked &&
                result.analysis.source == SignalAnalysisSource::demodulator,
            "GUI analysis did not switch to production demod telemetry");
    require(std::isfinite(result.analysis.cp_snr_db) &&
                std::isfinite(result.analysis.deepest_notch_db),
            "production signal analysis contains a non-finite value");
    require(result.analysis.cp_snr_db > 10.0F,
            "production CP SNR was not populated from the ideal signal");
    require(result.signal.signal_locked && result.signal.transport_locked,
            "common signal snapshot did not expose DVB-T lock state");
    require(result.signal.constellation_count != 0 &&
                result.signal.constellation_count <=
                    result.signal.constellation.size(),
            "common constellation snapshot has an invalid point count");
    require(result.signal.carrier_offset_limit_hz > 0.0F,
            "common signal snapshot lacks a carrier-offset scale");
    require(result.pipeline.stage_count == 3 &&
                result.pipeline.stages[0].name == "IQ queue" &&
                result.pipeline.stages[1].name == "Demod" &&
                result.pipeline.stages[2].name == "FEC queue",
            "common pipeline snapshot did not preserve DVB-T stage order");
    require(result.pipeline.stages[0].queue_valid &&
                result.pipeline.stages[1].busy_valid &&
                result.pipeline.stages[2].queue_valid,
            "common pipeline snapshot has invalid stage metric types");
    require(std::isfinite(result.stats.raw_timing_offset_samples) &&
                std::isfinite(result.stats.timing_offset_samples) &&
                std::isfinite(result.stats.physical_timing_offset_samples) &&
                std::isfinite(result.stats.sample_clock_offset_ppm) &&
                std::isfinite(result.stats.sro_resampler_command_ppm) &&
                std::isfinite(result.stats.sro_resampler_applied_ppm),
            "timing telemetry contains a non-finite value");
    require(result.stats.queued_blocks == 0 &&
                result.stats.queued_input_samples == 0 &&
                result.stats.queued_symbols == 0,
            "8K decoder queues did not drain");
    require(std::count(result.discontinuities.begin(),
                       result.discontinuities.end(),
                       TransportDiscontinuity::stream_end) == 1,
            "8K finite stream did not report exactly one stream end");
}

void test_8k_frontend_cfo_centering() {
    const auto encoded = encode_transport();
    const auto clean_iq = make_iq(encoded.metrics);
    constexpr std::array<std::size_t, 8> block_sizes{17,  4097,  12345, 8191,
                                                     777, 16384, 251,   10003};
    constexpr double subcarrier_spacing_hz =
        static_cast<double>(sample_rate) / static_cast<double>(fft_size);
    for (const auto [integer_bins, fractional_cfo_hz] :
         {std::pair{5, 237.25}, std::pair{-7, -311.75}}) {
        const double injected_cfo_hz =
            (static_cast<double>(integer_bins) * subcarrier_spacing_hz) +
            fractional_cfo_hz;
        const auto shifted_iq = apply_cfo(clean_iq, injected_cfo_hz);
        const auto serial = decode_in_blocks(shifted_iq, block_sizes, 1);
        const auto parallel = decode_in_blocks(shifted_iq, block_sizes, 8);
        require(has_known_run(serial.transport, encoded.transport),
                "frontend CFO centering produced no serial TS run");
        require(serial.transport == parallel.transport,
                "nonzero-CFO output changed with resampler worker count");
        require(serial.stats.carrier_bin_offset == 0 &&
                    parallel.stats.carrier_bin_offset == 0,
                "production demod retained an integer carrier offset");
        require(serial.stats.cfo_resampler_ready &&
                    parallel.stats.cfo_resampler_ready,
                "frontend CFO actuator did not become ready");
        require(serial.stats.bootstrap_attempts != 0 &&
                    serial.stats.bootstrap_replayed_input_samples != 0 &&
                    serial.stats.bootstrap_retained_peak_samples >=
                        serial.stats.bootstrap_replayed_input_samples,
                "frontend bootstrap replay telemetry is inconsistent");
        require(std::abs(static_cast<double>(serial.stats.acquisition_cfo_hz) -
                         injected_cfo_hz) < 30.0,
                "bootstrap acquisition CFO does not match injected CFO");
        require(serial.stats.acquisition_carrier_bin_offset == integer_bins &&
                    std::abs(static_cast<double>(
                                 serial.stats.acquisition_fractional_cfo_hz) -
                             fractional_cfo_hz) < 30.0,
                "bootstrap did not preserve integer/fractional CFO telemetry");
        require(std::abs(
                    static_cast<double>(serial.stats.cfo_resampler_applied_hz) -
                    injected_cfo_hz) < 30.0,
                "frontend resampler did not apply the acquired CFO");
        require(std::abs(serial.stats.residual_carrier_offset_hz) < 50.0F,
                "centered production stream retained excessive residual CFO");
    }
}

void test_8k_frontend_cfo_drift_tracking() {
    const auto encoded = encode_transport();
    const auto repeated_iq = repeat_iq(make_iq(encoded.metrics), 20);
    constexpr double subcarrier_spacing_hz =
        static_cast<double>(sample_rate) / static_cast<double>(fft_size);
    constexpr double initial_cfo_hz = (4.0 * subcarrier_spacing_hz) + 175.0;
    constexpr double drift_hz_per_second = 60.0;
    const auto shifted_iq =
        apply_cfo_drift(repeated_iq, initial_cfo_hz, drift_hz_per_second);
    constexpr std::array<std::size_t, 8> block_sizes{17,  4097,  12345, 8191,
                                                     777, 16384, 251,   10003};
    const auto result = decode_in_blocks(shifted_iq, block_sizes, 8);
    require(has_known_run(result.transport, encoded.transport),
            "slow CFO drift prevented TS recovery");
    require(result.stats.cfo_fixed_delay_samples != 0,
            "residual CFO tracker did not schedule a sample-domain command");
    require(result.stats.cfo_input_sample_rate_hz == sample_rate,
            "CFO scheduling telemetry lost the independent source rate");
    require(result.stats.cfo_applied_input_sample >=
                    result.stats.cfo_applied_effective_input_sample &&
                result.stats.cfo_applied_effective_input_sample != 0,
            "CFO correction did not honor its future activation sample");
    require(result.stats.cfo_schedule_late_samples ==
                result.stats.cfo_applied_input_sample -
                    result.stats.cfo_applied_effective_input_sample,
            "CFO late-scheduling telemetry is inconsistent");
    require(std::abs(result.stats.cfo_resampler_command_hz -
                     result.stats.acquisition_cfo_hz) > 5.0F,
            "CFO command did not follow the injected slow drift");
    require(std::abs(result.stats.cfo_resampler_applied_hz -
                     result.stats.acquisition_cfo_hz) > 5.0F,
            "scheduled CFO correction never reached the frontend resampler");
    require(std::abs(result.stats.residual_carrier_offset_hz) < 150.0F,
            "slow CFO drift escaped the residual tracking range");
}

void test_8k_frontend_abrupt_cfo_rebootstrap() {
    const auto encoded = encode_transport();
    constexpr std::size_t repetitions = 12;
    const auto repeated_iq = repeat_iq(make_iq(encoded.metrics), repetitions);
    constexpr double subcarrier_spacing_hz =
        static_cast<double>(sample_rate) / static_cast<double>(fft_size);
    constexpr double initial_cfo_hz = (4.0 * subcarrier_spacing_hz) + 175.0;
    constexpr double final_cfo_hz = (-3.0 * subcarrier_spacing_hz) - 225.0;
    const std::size_t step_sample = (repetitions / 2U) * symbols * symbol_size;
    const auto shifted_iq =
        apply_cfo_step(repeated_iq, step_sample, initial_cfo_hz, final_cfo_hz);
    constexpr std::array<std::size_t, 8> block_sizes{17,  4097,  12345, 8191,
                                                     777, 16384, 251,   10003};

    std::vector<std::uint8_t> output;
    std::vector<TransportDiscontinuity> discontinuities;
    std::size_t post_rebootstrap_offset = 0;
    std::mutex callback_mutex;
    StreamDecoder decoder;
    decoder.set_parameters({.channel_bandwidth_hz = channel_bandwidth,
                            .mode = TransmissionMode::k8,
                            .guard_interval = GuardInterval::gi_1_4,
                            .constellation = Constellation::qpsk,
                            .code_rate = CodeRate::rate_1_2,
                            .worker_threads = 8});
    decoder.set_transport_callback(
        [&output, &callback_mutex](const std::span<const std::uint8_t> bytes) {
            const std::scoped_lock lock(callback_mutex);
            output.insert(output.end(), bytes.begin(), bytes.end());
        });
    decoder.set_discontinuity_callback(
        [&output, &discontinuities, &post_rebootstrap_offset,
         &callback_mutex](const TransportDiscontinuity discontinuity) {
            const std::scoped_lock lock(callback_mutex);
            discontinuities.push_back(discontinuity);
            if (discontinuity == TransportDiscontinuity::retune) {
                post_rebootstrap_offset = output.size();
            }
        });

    submit_in_blocks(decoder, shifted_iq, block_sizes);
    decoder.flush();
    decoder.wait_until_idle();
    const auto stats = decoder.stats();
    std::span<const std::uint8_t> post_rebootstrap{output};
    if (post_rebootstrap_offset < output.size()) {
        post_rebootstrap = post_rebootstrap.subspan(post_rebootstrap_offset);
    }

    require(stats.cfo_rebootstrap_requests >= 1 &&
                stats.cfo_rebootstrap_count >= 1,
            "abrupt CFO did not return to frontend bootstrap");
    require(stats.cfo_rebootstrap_count <= stats.cfo_rebootstrap_requests,
            "CFO rebootstrap completion accounting is inconsistent");
    require(std::count(discontinuities.begin(), discontinuities.end(),
                       TransportDiscontinuity::retune) >= 1,
            "CFO rebootstrap did not publish a transport discontinuity");
    require(has_known_run(post_rebootstrap, encoded.transport),
            "TS output did not recover after abrupt CFO rebootstrap");
    require(stats.carrier_bin_offset == 0,
            "post-rebootstrap production demod retained a carrier offset");
    require(std::abs(static_cast<double>(stats.acquisition_cfo_hz) -
                     final_cfo_hz) < 35.0,
            "post-jump bootstrap did not acquire the new CFO");
    require(std::abs(static_cast<double>(stats.cfo_resampler_applied_hz) -
                     final_cfo_hz) < 50.0,
            "post-jump CFO was not applied by the frontend resampler");
    require(stats.cfo_rebootstrap_source_sample != 0 &&
                stats.cfo_rebootstrap_output_sample != 0,
            "CFO rebootstrap lost its sample-domain trigger coordinates");
    require(stats.processed_input_samples == repeated_iq.size() / 2U,
            "CFO rebootstrap lost or double-counted raw input samples");
}

void test_8k_reset_then_new_stream() {
    const auto encoded = encode_transport();
    const auto iq = make_iq(encoded.metrics);
    constexpr std::array<std::size_t, 5> block_sizes{4097, 8191, 12345, 777,
                                                     16384};

    std::vector<std::uint8_t> output;
    std::vector<TransportDiscontinuity> discontinuities;
    std::mutex callback_mutex;
    StreamDecoder decoder;
    decoder.set_parameters({.channel_bandwidth_hz = channel_bandwidth,
                            .mode = TransmissionMode::k8,
                            .guard_interval = GuardInterval::gi_1_4,
                            .constellation = Constellation::qpsk,
                            .code_rate = CodeRate::rate_1_2,
                            .worker_threads = 4});
    decoder.set_transport_callback(
        [&output, &callback_mutex](const std::span<const std::uint8_t> bytes) {
            const std::scoped_lock lock(callback_mutex);
            output.insert(output.end(), bytes.begin(), bytes.end());
        });
    decoder.set_discontinuity_callback(
        [&discontinuities,
         &callback_mutex](const TransportDiscontinuity discontinuity) {
            const std::scoped_lock lock(callback_mutex);
            discontinuities.push_back(discontinuity);
        });

    submit_in_blocks(decoder, iq, block_sizes);
    decoder.flush();
    decoder.wait_until_idle();
    const std::size_t first_stream_bytes = output.size();
    require(has_known_run(output, encoded.transport),
            "8K first stream did not produce a known TS run");

    decoder.reset();
    decoder.wait_until_idle();
    submit_in_blocks(decoder, iq, block_sizes);
    decoder.flush();
    decoder.wait_until_idle();

    const auto second_stream =
        std::span<const std::uint8_t>{output}.subspan(first_stream_bytes);
    require(has_known_run(second_stream, encoded.transport),
            "8K reset did not produce a clean second-stream TS run");
    require(std::count(discontinuities.begin(), discontinuities.end(),
                       TransportDiscontinuity::retune) >= 1,
            "reset did not propagate a retune discontinuity");
    require(std::count(discontinuities.begin(), discontinuities.end(),
                       TransportDiscontinuity::stream_end) == 2,
            "two finite 8K streams did not produce two stream-end events");
}

void test_8k_reset_after_hopeless_stream() {
    // Regression for file re-open after a decode failure. The first stream
    // has valid OFDM pilots but no payload energy, so it establishes several
    // hopeless MER windows. A subsequent reset must clear that recovery
    // state before the new stream's first pilot lock.
    const auto encoded = encode_transport();
    const auto clean_iq = make_iq(encoded.metrics);
    const auto hopeless_symbol_block = make_iq(encoded.metrics, 0.0F);
    std::vector<std::int16_t> hopeless_iq;
    hopeless_iq.reserve(hopeless_symbol_block.size() * 5);
    for (int copy = 0; copy < 5; ++copy) {
        hopeless_iq.insert(hopeless_iq.end(), hopeless_symbol_block.begin(),
                           hopeless_symbol_block.end());
    }
    constexpr std::array<std::size_t, 5> block_sizes{4097, 8191, 12345, 777,
                                                     16384};

    std::vector<std::uint8_t> output;
    std::mutex callback_mutex;
    StreamDecoder decoder;
    decoder.set_parameters({.channel_bandwidth_hz = channel_bandwidth,
                            .mode = TransmissionMode::k8,
                            .guard_interval = GuardInterval::gi_1_4,
                            .constellation = Constellation::qpsk,
                            .code_rate = CodeRate::rate_1_2,
                            .worker_threads = 4});
    decoder.set_transport_callback(
        [&output, &callback_mutex](const std::span<const std::uint8_t> bytes) {
            const std::scoped_lock lock(callback_mutex);
            output.insert(output.end(), bytes.begin(), bytes.end());
        });

    submit_in_blocks(decoder, hopeless_iq, block_sizes);
    decoder.flush();
    decoder.wait_until_idle();
    decoder.reset();

    submit_in_blocks(decoder, clean_iq, block_sizes);
    decoder.flush();
    decoder.wait_until_idle();
    require(has_known_run(output, encoded.transport),
            "8K reset after a hopeless stream did not recover cleanly");
}

void test_8k_automatic_manual_transitions() {
    constexpr std::array<std::size_t, 5> block_sizes{4097, 8191, 12345, 777,
                                                     16384};
    constexpr std::size_t tps_preamble_symbols = 40;
    constexpr std::size_t transition_symbols = tps_preamble_symbols +
                                                (2 * symbols);
    constexpr std::array<std::uint8_t, 4> salts{0x11, 0x43, 0x79, 0xB5};

    std::vector<std::uint8_t> output;
    std::vector<TransportDiscontinuity> discontinuities;
    std::mutex callback_mutex;
    std::condition_variable callback_changed;
    StreamDecoder decoder;
    decoder.set_transport_callback(
        [&output, &callback_mutex,
         &callback_changed](const std::span<const std::uint8_t> bytes) {
            {
                const std::scoped_lock lock(callback_mutex);
                output.insert(output.end(), bytes.begin(), bytes.end());
            }
            callback_changed.notify_all();
        });
    decoder.set_discontinuity_callback(
        [&discontinuities, &callback_mutex,
         &callback_changed](const TransportDiscontinuity discontinuity) {
            {
                const std::scoped_lock lock(callback_mutex);
                discontinuities.push_back(discontinuity);
            }
            callback_changed.notify_all();
        });

    const auto parameters_for = [](const std::size_t phase) {
        airspy_tv::dvbt::ReceiverParameters parameters;
        parameters.channel_bandwidth_hz = channel_bandwidth;
        parameters.worker_threads = 8;
        if (phase == 1) {
            parameters.mode = TransmissionMode::k8;
            parameters.guard_interval = GuardInterval::gi_1_4;
            parameters.constellation = Constellation::qpsk;
            parameters.code_rate = CodeRate::rate_1_2;
        } else if (phase == 2) {
            parameters.mode = TransmissionMode::k8;
            parameters.guard_interval = GuardInterval::gi_1_4;
        }
        return parameters;
    };

    std::vector<std::uint8_t> previous_transport;
    std::uint64_t previous_generation = decoder.stats().decoder_generation;
    for (std::size_t phase = 0; phase < salts.size(); ++phase) {
        decoder.set_parameters(parameters_for(phase));
        const auto generation = decoder.stats().decoder_generation;
        require(generation > previous_generation,
                "parameter transition did not advance decoder generation");
        previous_generation = generation;

        std::size_t boundary = 0;
        {
            const std::scoped_lock lock(callback_mutex);
            boundary = output.size();
        }
        const auto encoded = encode_transport(salts[phase]);
        const auto iq =
            make_iq(encoded.metrics, 1.0F, true, transition_symbols,
                    tps_preamble_symbols);
        if (phase == 0) {
            const auto resampled = airspy_tv::dvbt::resample_cs16(
                iq, sample_rate, channel_bandwidth, 2);
            const auto acquisition = airspy_tv::dvbt::acquire_ofdm(
                std::span<const std::complex<float>>{resampled}.first(
                    std::min<std::size_t>(350'000, resampled.size())),
                parameters_for(phase), true);
            if (acquisition.carrier_offset != 0) {
                std::cerr << "valid-TPS acquisition carrier offset: "
                          << acquisition.carrier_offset << '\n';
            }
            require(acquisition.score >= 0.20F &&
                        acquisition.mode == TransmissionMode::k8 &&
                        acquisition.guard == GuardInterval::gi_1_4 &&
                        acquisition.carrier_offset == 0,
                    "valid-TPS helper is not robustly auto-acquirable");
        }
        submit_in_blocks(decoder, iq, block_sizes);

        std::unique_lock lock(callback_mutex);
        const bool decoded = callback_changed.wait_for(
            lock, std::chrono::seconds(30), [&] {
                return has_known_run(std::span<const std::uint8_t>{output}
                                         .subspan(boundary),
                                     encoded.transport);
            });
        if (!decoded) {
            const auto stats = decoder.stats();
            std::cerr << "transition phase " << phase
                      << " timed out: output=" << output.size()
                      << " generation=" << stats.decoder_generation
                      << " ofdm=" << stats.ofdm_locked
                      << " tps=" << stats.tps_locked
                      << " tps-ever=" << stats.tps_ever_locked
                      << " attempts=" << stats.bootstrap_attempts
                      << " input=" << stats.processed_input_samples
                      << " queued-input=" << stats.queued_input_samples
                      << " ring=" << stats.ring_used_samples
                      << " queued-symbols=" << stats.queued_symbols << '\n';
        }
        require(decoded,
                "automatic/manual transition did not resume correct TS");
        const auto current =
            std::span<const std::uint8_t>{output}.subspan(boundary);
        if (!previous_transport.empty()) {
            require(!has_known_run(current, previous_transport),
                    "stale generation TS crossed a reset boundary");
        }
        require(std::count(discontinuities.begin(), discontinuities.end(),
                           TransportDiscontinuity::retune) >=
                    static_cast<std::ptrdiff_t>(phase + 1),
                "parameter transition did not publish retune discontinuity");
        previous_transport = encoded.transport;
    }

    decoder.flush();
    decoder.wait_until_idle();
    const auto stats = decoder.stats();
    require(stats.tps_ever_locked && stats.tps_locked,
            "valid synthetic TPS did not lock automatic parameters");
    require(stats.tps_mode == TransmissionMode::k8 &&
                stats.tps_guard_interval == GuardInterval::gi_1_4 &&
                stats.tps_constellation == Constellation::qpsk &&
                stats.tps_code_rate == CodeRate::rate_1_2,
            "automatic parameters did not match synthetic TPS");
    require(std::count(discontinuities.begin(), discontinuities.end(),
                       TransportDiscontinuity::stream_end) == 1,
            "transition stream did not finish exactly once");
}

void test_8k_submit_reset_stress() {
    const auto encoded = encode_transport(0xD3);
    const auto iq = make_iq(encoded.metrics);
    constexpr std::array<std::size_t, 5> block_sizes{4097, 8191, 12345, 777,
                                                     16384};
    constexpr std::size_t iterations = 24;

    std::vector<std::uint8_t> output;
    std::mutex callback_mutex;
    StreamDecoder decoder;
    airspy_tv::dvbt::ReceiverParameters parameters;
    parameters.channel_bandwidth_hz = channel_bandwidth;
    parameters.mode = TransmissionMode::k8;
    parameters.guard_interval = GuardInterval::gi_1_4;
    parameters.constellation = Constellation::qpsk;
    parameters.code_rate = CodeRate::rate_1_2;
    parameters.worker_threads = 4;
    decoder.set_parameters(parameters);
    decoder.set_transport_callback(
        [&output, &callback_mutex](const std::span<const std::uint8_t> bytes) {
            const std::scoped_lock lock(callback_mutex);
            output.insert(output.end(), bytes.begin(), bytes.end());
        });

    std::barrier operation_start{2};
    std::barrier operation_done{2};
    std::thread producer([&] {
        std::uint32_t state = 0xC001D00DU;
        for (std::size_t iteration = 0; iteration < iterations; ++iteration) {
            operation_start.arrive_and_wait();
            state = state * 1664525U + 1013904223U;
            const std::size_t begin =
                (static_cast<std::size_t>(state) %
                 (iq.size() / 2U - symbol_size)) *
                2U;
            decoder.submit(std::span<const std::int16_t>{iq}.subspan(
                               begin, symbol_size * 2U),
                           sample_rate, channel_bandwidth);
            operation_done.arrive_and_wait();
        }
    });
    std::thread controller([&] {
        for (std::size_t iteration = 0; iteration < iterations; ++iteration) {
            operation_start.arrive_and_wait();
            if ((iteration & 1U) == 0U) {
                decoder.request_reset();
            } else {
                decoder.reset();
            }
            operation_done.arrive_and_wait();
        }
    });
    producer.join();
    controller.join();

    // Establish a clean generation, then model a finite source: its producer
    // is joined before flush is allowed to close and drain the stream.
    decoder.reset();
    std::size_t boundary = 0;
    {
        const std::scoped_lock lock(callback_mutex);
        boundary = output.size();
    }
    std::thread finite_producer(
        [&] { submit_in_blocks(decoder, iq, block_sizes); });
    finite_producer.join();
    decoder.flush();
    decoder.wait_until_idle();

    const std::scoped_lock lock(callback_mutex);
    require(has_known_run(
                std::span<const std::uint8_t>{output}.subspan(boundary),
                encoded.transport),
            "decoder did not recover after submit/reset stress");
    require(decoder.stats().queued_blocks == 0,
            "finite producer did not drain before lifecycle completion");
}

void test_8k_non_acquirable_stream_drains() {
    const std::vector<std::int16_t> silence(symbols * symbol_size * 2, 0);
    constexpr std::array<std::size_t, 3> block_sizes{7001, 12003, 4099};

    StreamDecoder decoder;
    decoder.set_parameters({.channel_bandwidth_hz = channel_bandwidth,
                            .mode = TransmissionMode::k8,
                            .guard_interval = GuardInterval::gi_1_4,
                            .constellation = Constellation::qpsk,
                            .code_rate = CodeRate::rate_1_2,
                            .worker_threads = 2});
    submit_in_blocks(decoder, silence, block_sizes);
    decoder.flush();
    decoder.wait_until_idle();
    const auto stats = decoder.stats();
    require(!stats.ofdm_locked,
            "zero-I/Q non-acquirable stream falsely acquired OFDM");
    require(stats.bootstrap_attempts != 0 &&
                stats.bootstrap_retained_peak_samples < 500'000,
            "non-acquirable bootstrap did not retain a bounded raw window");
    require(stats.queued_blocks == 0 && stats.queued_input_samples == 0 &&
                stats.queued_symbols == 0,
            "non-acquirable 8K stream did not drain its queues");
}

void test_8k_reset_live_resume() {
    // GUI live-session shape: a running stream is torn down with reset()
    // and the *next* session starts submitting immediately — no flush and
    // no wait_until_idle between stop and re-open. The reset must still
    // produce a clean second run from the same signal.
    const auto encoded = encode_transport();
    const auto iq = make_iq(encoded.metrics);
    constexpr std::array<std::size_t, 5> block_sizes{4097, 8191, 12345, 777,
                                                     16384};

    std::vector<std::uint8_t> output;
    std::mutex callback_mutex;
    StreamDecoder decoder;
    decoder.set_parameters({.channel_bandwidth_hz = channel_bandwidth,
                            .mode = TransmissionMode::k8,
                            .guard_interval = GuardInterval::gi_1_4,
                            .constellation = Constellation::qpsk,
                            .code_rate = CodeRate::rate_1_2,
                            .worker_threads = 4});
    decoder.set_transport_callback(
        [&output, &callback_mutex](const std::span<const std::uint8_t> bytes) {
            const std::scoped_lock lock(callback_mutex);
            output.insert(output.end(), bytes.begin(), bytes.end());
        });

    // First session: feed the first half, then tear down mid-stream (the
    // GUI "close device" path) without draining.
    const std::size_t half_samples = iq.size() / 2;
    submit_in_blocks(decoder,
                     std::span<const std::int16_t>{iq}.first(half_samples),
                     block_sizes);
    decoder.reset();

    // Second session: immediately re-submit the full signal, exactly as a
    // re-opened SDR source would, then end the stream.
    submit_in_blocks(decoder, iq, block_sizes);
    decoder.flush();
    decoder.wait_until_idle();

    require(has_known_run(output, encoded.transport),
            "8K live reset did not produce a clean TS run after re-open");
}

void test_8k_reset_while_demod_waiting() {
    // Rapid retune shape: the source stalls briefly (hardware frequency
    // change), the demod consumes the ring down to nothing and parks in the
    // inner ring_data.wait for the next symbol, and the reset lands while it
    // is parked there. The wait must wake on the sync-version bump even
    // though the rewound ring can never satisfy the stale read position;
    // otherwise the demod blocks forever, the ring backs up, and live
    // submit() starts dropping blocks (rising dropped_blocks, idle threads).
    const auto encoded = encode_transport();
    const auto iq = make_iq(encoded.metrics);
    constexpr std::array<std::size_t, 3> block_sizes{12345, 8191, 16384};

    std::vector<std::uint8_t> output;
    std::mutex callback_mutex;
    StreamDecoder decoder;
    decoder.set_parameters({.channel_bandwidth_hz = channel_bandwidth,
                            .mode = TransmissionMode::k8,
                            .guard_interval = GuardInterval::gi_1_4,
                            .constellation = Constellation::qpsk,
                            .code_rate = CodeRate::rate_1_2,
                            .worker_threads = 4});
    decoder.set_transport_callback(
        [&output, &callback_mutex](const std::span<const std::uint8_t> bytes) {
            const std::scoped_lock lock(callback_mutex);
            output.insert(output.end(), bytes.begin(), bytes.end());
        });

    // First session: 40 symbols — enough to trigger acquisition (the demod
    // waits for acquisition_samples before its first anchor) but not to
    // finish a TPS frame, so after the demod drains the ring it parks in the
    // inner ring_data.wait for the next symbol (live source stall / rapid
    // retune shape; no flush, no wait_until_idle).
    const std::size_t first_scalars =
        static_cast<std::size_t>(40) * symbol_size * 2;
    submit_in_blocks(decoder,
                     std::span<const std::int16_t>{iq}.first(first_scalars),
                     block_sizes);
    // Let the front-end consume the blocks and the demod drain the ring down
    // to nothing, so it parks in the inner ring_data.wait for the next
    // symbol (live source stall / rapid retune shape; no flush, no
    // wait_until_idle).
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    decoder.reset();

    // Second session: the full signal, as a re-opened source would send it.
    submit_in_blocks(decoder, iq, block_sizes);
    decoder.flush();
    decoder.wait_until_idle();
    require(has_known_run(output, encoded.transport),
            "8K mid-wait reset did not produce a clean TS run after resume");
}

} // namespace

int main() {
    try {
        const std::string_view selected = [] {
            const char *value = std::getenv("AIRSPYTV_STREAM_TEST");
            return value == nullptr ? std::string_view{}
                                    : std::string_view{value};
        }();
        bool ran = false;
        const auto run = [selected, &ran](const std::string_view name,
                                          const auto &test) {
            if (selected.empty() || selected == name) {
                ran = true;
                test();
            }
        };
        run("clean", test_8k_clean_signal);
        run("cfo-centering", test_8k_frontend_cfo_centering);
        run("cfo-drift", test_8k_frontend_cfo_drift_tracking);
        run("cfo-rebootstrap", test_8k_frontend_abrupt_cfo_rebootstrap);
        run("worker-parity", test_8k_worker_count_parity);
        run("queue-capacity", test_8k_queue_capacity_multiplier);
        run("reset-new", test_8k_reset_then_new_stream);
        run("reset-hopeless", test_8k_reset_after_hopeless_stream);
        run("mode-transition", test_8k_automatic_manual_transitions);
        run("lifecycle-stress", test_8k_submit_reset_stress);
        run("reset-live", test_8k_reset_live_resume);
        run("reset-wait", test_8k_reset_while_demod_waiting);
        run("no-acq", test_8k_non_acquirable_stream_drains);
        require(ran, "unknown AIRSPYTV_STREAM_TEST selection");
    } catch (const std::exception &error) {
        std::cerr << "DVB-T StreamDecoder integration test failed: "
                  << error.what() << '\n';
        return 1;
    }
    std::cout << "DVB-T StreamDecoder integration tests passed\n";
    return 0;
}
