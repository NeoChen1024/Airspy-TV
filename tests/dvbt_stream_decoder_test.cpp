#include "airspy_tv/dvbt/stream_decoder.hpp"

#include "airspy_tv/dvbt/inner_decoder.hpp"
#include "airspy_tv/dvbt/soft_demapper.hpp"

extern "C" {
#include <correct.h>
}
#include <fftw3.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <mutex>
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
using airspy_tv::dvbt::StreamDecoder;
using airspy_tv::dvbt::SymbolDeinterleaver;
using airspy_tv::dvbt::TransmissionMode;

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
            ((shift_register << 1U) | feedback) & 0x7FFFU);
        result = static_cast<std::uint8_t>((result << 1U) | feedback);
    }
    return result;
}

[[nodiscard]] std::vector<std::uint8_t>
make_transport_stream(const std::size_t count) {
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
                (packet * 29U + byte * 17U + 3U) & 0xFFU);
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
            const std::size_t offset = group + packet * ts_packet_size;
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
    correct_reed_solomon *codec = correct_reed_solomon_create(
        correct_rs_primitive_polynomial_8_4_3_2_0, 0, 1, 16);
    require(codec != nullptr, "create RS encoder");
    std::vector<std::uint8_t> output((randomized.size() / ts_packet_size) *
                                     rs_packet_size);
    for (std::size_t input = 0, encoded = 0; input < randomized.size();
         input += ts_packet_size, encoded += rs_packet_size) {
        const ssize_t size = correct_reed_solomon_encode(
            codec, randomized.data() + input, ts_packet_size,
            output.data() + encoded);
        require(size == 255, "RS encoder returned an unexpected length");
    }
    correct_reed_solomon_destroy(codec);
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
                ((shift_register << 1U) | ((value >> bit) & 1U)) & 0x7FU);
            for (const std::uint8_t polynomial : polynomials) {
                output.push_back(static_cast<std::uint8_t>(
                    std::popcount(static_cast<unsigned int>(shift_register &
                                                            polynomial)) &
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
                    (output_bit / half) + 2 * (output_bit % half);
                const std::size_t source_position =
                    (position + block_carriers - offsets[source_bit]) %
                    block_carriers;
                output[block + source_position * bits_per_carrier +
                       source_bit] =
                    input[block + position * bits_per_carrier + output_bit];
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

[[nodiscard]] EncodedStream encode_transport() {
    EncodedStream result;
    result.transport = make_transport_stream(packet_count);
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

[[nodiscard]] std::vector<std::int16_t>
make_iq(const std::span<const float> metrics) {
    const MaxLogDemapper demapper{Constellation::qpsk};
    const SymbolDeinterleaver symbol_permutation{TransmissionMode::k8};
    const auto points = demapper.constellation_points();
    fftwf_complex *frequency = static_cast<fftwf_complex *>(
        fftwf_malloc(sizeof(fftwf_complex) * fft_size));
    fftwf_complex *time = static_cast<fftwf_complex *>(
        fftwf_malloc(sizeof(fftwf_complex) * fft_size));
    require(frequency != nullptr && time != nullptr, "allocate OFDM FFT");
    fftwf_plan plan = fftwf_plan_dft_1d(static_cast<int>(fft_size), frequency,
                                        time, FFTW_BACKWARD, FFTW_ESTIMATE);
    require(plan != nullptr, "create OFDM IFFT");

    std::vector<std::int16_t> result;
    result.reserve(symbols * symbol_size * 2);
    for (std::size_t symbol = 0; symbol < symbols; ++symbol) {
        std::fill(reinterpret_cast<float *>(frequency),
                  reinterpret_cast<float *>(frequency) + 2 * fft_size, 0.0F);
        const std::size_t phase = symbol % 4;
        const auto payload = payload_indices(phase);
        const auto symbol_metrics =
            metrics.subspan(symbol * payload_carriers * bits_per_carrier,
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
            } else if (!tps) {
                std::size_t label = 0;
                for (std::size_t bit = 0; bit < bits_per_carrier; ++bit) {
                    label = (label << 1U) |
                            (transmitted[payload_position * bits_per_carrier +
                                         bit] > 0.0F
                                 ? 1U
                                 : 0U);
                }
                value = points[label];
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
            if (symbol != 0) {
                const float noise =
                    0.002F * std::sin(static_cast<float>(index) * 0.071F +
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

struct DecodeResult {
    std::vector<std::uint8_t> transport;
    std::vector<TransportDiscontinuity> discontinuities;
    airspy_tv::dvbt::StreamDecoderStats stats;
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
                 const std::span<const std::size_t> block_sizes) {
    DecodeResult result;
    std::mutex callback_mutex;
    StreamDecoder decoder;
    decoder.set_parameters({.channel_bandwidth_hz = channel_bandwidth,
                            .mode = TransmissionMode::k8,
                            .guard_interval = GuardInterval::gi_1_4,
                            .constellation = Constellation::qpsk,
                            .code_rate = CodeRate::rate_1_2,
                            .worker_threads = 8});
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
    }
    return result;
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
    require(std::isfinite(result.stats.raw_timing_offset_samples) &&
                std::isfinite(result.stats.timing_offset_samples) &&
                std::isfinite(result.stats.physical_timing_offset_samples) &&
                std::isfinite(result.stats.sample_clock_offset_ppm) &&
                std::isfinite(result.stats.timing_shift_rate_ppm) &&
                std::isfinite(
                    result.stats.rolling_timing_shift_rate_ppm),
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
        test_8k_clean_signal();
        test_8k_reset_then_new_stream();
        test_8k_reset_live_resume();
        test_8k_reset_while_demod_waiting();
        test_8k_non_acquirable_stream_drains();
    } catch (const std::exception &error) {
        std::cerr << "DVB-T StreamDecoder integration test failed: "
                  << error.what() << '\n';
        return 1;
    }
    std::cout << "DVB-T StreamDecoder integration tests passed\n";
    return 0;
}
