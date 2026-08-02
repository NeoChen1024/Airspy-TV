#!/usr/bin/env python3

"""Extract an MPEG transport stream from a centered CS16 DVB-T capture."""

from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path

from gnuradio import blocks, dtv, fft, filter, gr
from gnuradio.fft import window


INPUT_SAMPLE_RATE = 10_000_000
INTERPOLATION = 24
DECIMATION = 35
FFT_LENGTH = 8192
OCCUPIED_TONES = 6817
PAYLOAD_CARRIERS = 6048

CONSTELLATIONS = {
    "qpsk": dtv.dvb_constellation_t.MOD_QPSK,
    "16qam": dtv.dvb_constellation_t.MOD_16QAM,
    "64qam": dtv.dvb_constellation_t.MOD_64QAM,
}
CODE_RATES = {
    "1/2": dtv.dvb_code_rate_t.C1_2,
    "2/3": dtv.dvb_code_rate_t.C2_3,
    "3/4": dtv.dvb_code_rate_t.C3_4,
    "5/6": dtv.dvb_code_rate_t.C5_6,
    "7/8": dtv.dvb_code_rate_t.C7_8,
}
GUARD_INTERVALS = {
    "1/32": (dtv.dvb_guardinterval_t.GI_1_32, FFT_LENGTH // 32),
    "1/16": (dtv.dvb_guardinterval_t.GI_1_16, FFT_LENGTH // 16),
    "1/8": (dtv.dvb_guardinterval_t.GI_1_8, FFT_LENGTH // 8),
    "1/4": (dtv.dvb_guardinterval_t.GI_1_4, FFT_LENGTH // 4),
}


class DvbtExtractor(gr.top_block):
    def __init__(
        self,
        source: Path,
        output: Path,
        offset_seconds: float,
        duration_seconds: float,
        constellation_name: str,
        code_rate_name: str,
        guard_interval_name: str,
        acquired_output: Path | None = None,
        equalized_output: Path | None = None,
    ) -> None:
        super().__init__("DVB-T CS16 extractor", catch_exceptions=True)

        constellation = CONSTELLATIONS[constellation_name]
        code_rate = CODE_RATES[code_rate_name]
        guard_interval, cp_length = GUARD_INTERVALS[guard_interval_name]
        transmission_mode = dtv.dvbt_transmission_mode_t.T8k
        hierarchy = dtv.dvbt_hierarchy_t.NH

        complex_offset = round(offset_seconds * INPUT_SAMPLE_RATE)
        complex_length = round(duration_seconds * INPUT_SAMPLE_RATE)
        source_block = blocks.file_source(
            gr.sizeof_short,
            str(source),
            False,
            complex_offset * 2,
            complex_length * 2,
        )
        to_complex = blocks.interleaved_short_to_complex(
            False, False, 32768.0
        )
        resampler = filter.rational_resampler_ccc(
            interpolation=INTERPOLATION,
            decimation=DECIMATION,
        )
        acquisition = dtv.dvbt_ofdm_sym_acquisition(
            1, FFT_LENGTH, OCCUPIED_TONES, cp_length, 30.0
        )
        fft_block = fft.fft_vcc(
            FFT_LENGTH,
            True,
            window.rectangular(FFT_LENGTH),
            True,
            1,
        )
        reference_demod = dtv.dvbt_demod_reference_signals(
            gr.sizeof_gr_complex,
            FFT_LENGTH,
            PAYLOAD_CARRIERS,
            constellation,
            hierarchy,
            code_rate,
            code_rate,
            guard_interval,
            transmission_mode,
            1,
            0,
        )
        demapper = dtv.dvbt_demap(
            PAYLOAD_CARRIERS,
            constellation,
            hierarchy,
            transmission_mode,
            1.0,
        )
        symbol_deinterleaver = dtv.dvbt_symbol_inner_interleaver(
            PAYLOAD_CARRIERS, transmission_mode, 0
        )
        bit_deinterleaver = dtv.dvbt_bit_inner_deinterleaver(
            PAYLOAD_CARRIERS,
            constellation,
            hierarchy,
            transmission_mode,
        )
        vector_to_stream = blocks.vector_to_stream(
            gr.sizeof_char, PAYLOAD_CARRIERS
        )
        viterbi = dtv.dvbt_viterbi_decoder(
            constellation, hierarchy, code_rate, 768
        )
        convolutional_deinterleaver = dtv.dvbt_convolutional_deinterleaver(
            136, 12, 17
        )
        reed_solomon = dtv.dvbt_reed_solomon_dec(
            2, 8, 0x11D, 255, 239, 8, 51, 8
        )
        descrambler = dtv.dvbt_energy_descramble(8)
        sink = blocks.file_sink(gr.sizeof_char, str(output), False)

        self.connect(source_block, to_complex, resampler, acquisition)
        self.connect(acquisition, fft_block, reference_demod, demapper)
        if acquired_output is not None:
            acquired_stream = blocks.vector_to_stream(
                gr.sizeof_gr_complex, FFT_LENGTH
            )
            acquired_sink = blocks.file_sink(
                gr.sizeof_gr_complex, str(acquired_output), False
            )
            self.connect(acquisition, acquired_stream, acquired_sink)
        if equalized_output is not None:
            equalized_stream = blocks.vector_to_stream(
                gr.sizeof_gr_complex, PAYLOAD_CARRIERS
            )
            equalized_sink = blocks.file_sink(
                gr.sizeof_gr_complex, str(equalized_output), False
            )
            self.connect(reference_demod, equalized_stream, equalized_sink)
        self.connect(
            demapper,
            symbol_deinterleaver,
            bit_deinterleaver,
            vector_to_stream,
            viterbi,
            convolutional_deinterleaver,
            reed_solomon,
            descrambler,
            sink,
        )


def report_transport_stream(path: Path) -> bool:
    size = path.stat().st_size if path.exists() else 0
    if size == 0:
        print("output: empty")
        return False

    probe = path.read_bytes()[: 188 * 20_000]
    packet_count = len(probe) // 188
    sync_count = 0
    transport_errors = 0
    pids: Counter[int] = Counter()
    valid_pat = False
    programs: dict[int, int] = {}
    for index in range(packet_count):
        packet = probe[index * 188 : (index + 1) * 188]
        if packet[0] != 0x47:
            continue
        sync_count += 1
        transport_errors += bool(packet[1] & 0x80)
        pid = ((packet[1] & 0x1F) << 8) | packet[2]
        pids[pid] += 1
        payload_start = bool(packet[1] & 0x40)
        adaptation = (packet[3] >> 4) & 0x03
        if pid != 0 or not payload_start or adaptation not in (1, 3):
            continue
        payload_offset = 4
        if adaptation == 3:
            payload_offset += 1 + packet[4]
        if payload_offset >= 188:
            continue
        pointer = packet[payload_offset]
        section_start = payload_offset + 1 + pointer
        if section_start + 8 > 188 or packet[section_start] != 0x00:
            continue
        section_length = (
            ((packet[section_start + 1] & 0x0F) << 8)
            | packet[section_start + 2]
        )
        section_end = section_start + 3 + section_length
        if section_length < 9 or section_end > 188:
            continue
        entries_end = section_end - 4
        if (entries_end - (section_start + 8)) % 4 != 0:
            continue
        for pos in range(section_start + 8, entries_end, 4):
            program = (packet[pos] << 8) | packet[pos + 1]
            program_pid = ((packet[pos + 2] & 0x1F) << 8) | packet[pos + 3]
            if program != 0:
                programs[program] = program_pid
        valid_pat = bool(programs)
    print(
        f"output: {size} bytes; sync bytes at 188-byte boundaries: "
        f"{sync_count}/{packet_count}"
    )
    print(
        f"transport errors: {transport_errors}; "
        f"top PIDs: {pids.most_common(8)}; PAT programs: {programs}"
    )
    return (
        packet_count > 0
        and sync_count == packet_count
        and transport_errors < packet_count // 100
        and valid_pat
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--offset", type=float, default=2.0)
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument(
        "--constellation", choices=CONSTELLATIONS, default="64qam"
    )
    parser.add_argument("--code-rate", choices=CODE_RATES, default="2/3")
    parser.add_argument(
        "--guard-interval", choices=GUARD_INTERVALS, default="1/4"
    )
    parser.add_argument("--acquired-output", type=Path)
    parser.add_argument("--equalized-output", type=Path)
    args = parser.parse_args()

    if not args.source.is_file():
        parser.error(f"source does not exist: {args.source}")
    if args.offset < 0.0 or args.duration <= 0.0:
        parser.error("offset must be non-negative and duration positive")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    print(
        f"decoding {args.duration:g}s at +{args.offset:g}s: "
        f"8K {args.constellation} rate={args.code_rate} "
        f"GI={args.guard_interval}"
    )
    flowgraph = DvbtExtractor(
        args.source,
        args.output,
        args.offset,
        args.duration,
        args.constellation,
        args.code_rate,
        args.guard_interval,
        args.acquired_output,
        args.equalized_output,
    )
    flowgraph.run()
    return 0 if report_transport_stream(args.output) else 1


if __name__ == "__main__":
    raise SystemExit(main())
