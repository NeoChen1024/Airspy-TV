#include "airspy_tv/dvbt/stream_decoder.hpp"

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <span>
#include <thread>
#include <vector>

int main(const int argc, const char *const argv[]) {
    if (argc < 3 || argc > 4) {
        std::cerr << "usage: airspy-tv-dvbt-iq INPUT.cs16 OUTPUT.ts [CHUNKS]\n";
        return 1;
    }
    const std::size_t limit = argc == 4 ? std::stoul(argv[3]) : 3;
    std::ifstream input(argv[1], std::ios::binary);
    std::ofstream output(argv[2], std::ios::binary | std::ios::trunc);
    if (!input || !output) {
        std::cerr << "failed to open input or output\n";
        return 1;
    }
    airspy_tv::dvbt::StreamDecoder decoder;
    decoder.set_transport_callback(
        [&output](const std::span<const std::uint8_t> ts) {
            output.write(reinterpret_cast<const char *>(ts.data()),
                         static_cast<std::streamsize>(ts.size()));
        });
    constexpr std::size_t complex_samples = 1'400'000;
    std::vector<std::int16_t> block(complex_samples * 2);
    for (std::size_t chunk = 0; chunk < limit; ++chunk) {
        input.read(
            reinterpret_cast<char *>(block.data()),
            static_cast<std::streamsize>(block.size() * sizeof(block[0])));
        if (input.gcount() !=
            static_cast<std::streamsize>(block.size() * sizeof(block[0]))) {
            break;
        }
        const auto previous = decoder.stats().ofdm_symbols;
        decoder.submit(block, 10'000'000);
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(30);
        while (decoder.stats().ofdm_symbols == previous &&
               std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        const auto stats = decoder.stats();
        std::cerr << "chunk=" << chunk + 1 << " symbols=" << stats.ofdm_symbols
                  << " TS=" << stats.transport_bytes << " bytes"
                  << " acq=" << stats.acquisition_score
                  << " N=" << stats.fft_size << "+" << stats.guard_size
                  << " bins=" << stats.carrier_bin_offset
                  << " MER=" << stats.mer_db
                  << " phase slips=" << stats.pilot_phase_discontinuities
                  << '\n';
    }
    output.flush();
    const auto stats = decoder.stats();
    std::cerr << "Viterbi bits=" << stats.transport.viterbi_bits
              << " outer phase=" << stats.transport.outer_deinterleaver_phase
              << " RS=" << stats.transport.rs_packets
              << " failures=" << stats.transport.rs_uncorrectable_packets
              << " packets=" << stats.transport.ts_packets << '\n';
    return stats.transport_bytes == 0 ? 2 : 0;
}
