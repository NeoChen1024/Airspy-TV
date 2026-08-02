#!/usr/bin/env python3

"""Decode GNU Radio gr-dtv equalized DVB-T payload carriers."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pmt
from gnuradio import blocks, dtv, gr

from dvbt_extract import PAYLOAD_CARRIERS, report_transport_stream


class SymbolIndexTagger(gr.sync_block):
    """Restore the per-OFDM-symbol tags expected by gr-dtv's deinterleaver."""

    def __init__(self, first_symbol: int) -> None:
        super().__init__(
            name="DVB-T symbol index tagger",
            in_sig=[(np.complex64, PAYLOAD_CARRIERS)],
            out_sig=[(np.complex64, PAYLOAD_CARRIERS)],
        )
        self._symbol = first_symbol % 68

    def work(self, input_items, output_items):
        count = len(input_items[0])
        output_items[0][:count] = input_items[0][:count]
        base = self.nitems_written(0)
        for offset in range(count):
            self.add_item_tag(
                0,
                base + offset,
                pmt.intern("symbol_index"),
                pmt.from_long(self._symbol),
            )
            self._symbol = (self._symbol + 1) % 68
        return count


class EqualizedDecoder(gr.top_block):
    def __init__(
        self, source: Path, output: Path, first_symbol: int, gain: float
    ) -> None:
        super().__init__("DVB-T equalized hard decoder", catch_exceptions=True)
        constellation = dtv.dvb_constellation_t.MOD_64QAM
        hierarchy = dtv.dvbt_hierarchy_t.NH
        code_rate = dtv.dvb_code_rate_t.C2_3
        transmission = dtv.dvbt_transmission_mode_t.T8k

        source_block = blocks.file_source(gr.sizeof_gr_complex, str(source), False)
        to_vector = blocks.stream_to_vector(gr.sizeof_gr_complex, PAYLOAD_CARRIERS)
        # Keep the Python block alive for as long as the C++ flowgraph uses it.
        self.tagger = SymbolIndexTagger(first_symbol)
        demapper = dtv.dvbt_demap(
            PAYLOAD_CARRIERS, constellation, hierarchy, transmission, gain
        )
        symbol_deinterleaver = dtv.dvbt_symbol_inner_interleaver(
            PAYLOAD_CARRIERS, transmission, 0
        )
        bit_deinterleaver = dtv.dvbt_bit_inner_deinterleaver(
            PAYLOAD_CARRIERS, constellation, hierarchy, transmission
        )
        to_stream = blocks.vector_to_stream(gr.sizeof_char, PAYLOAD_CARRIERS)
        viterbi = dtv.dvbt_viterbi_decoder(
            constellation, hierarchy, code_rate, 768
        )
        byte_deinterleaver = dtv.dvbt_convolutional_deinterleaver(136, 12, 17)
        reed_solomon = dtv.dvbt_reed_solomon_dec(
            2, 8, 0x11D, 255, 239, 8, 51, 8
        )
        descrambler = dtv.dvbt_energy_descramble(8)
        sink = blocks.file_sink(gr.sizeof_char, str(output), False)

        self.connect(
            source_block,
            to_vector,
            self.tagger,
            demapper,
            symbol_deinterleaver,
            bit_deinterleaver,
            to_stream,
            viterbi,
            byte_deinterleaver,
            reed_solomon,
            descrambler,
            sink,
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--first-symbol", type=int, default=0)
    parser.add_argument("--gain", type=float, default=1.0)
    args = parser.parse_args()

    flowgraph = EqualizedDecoder(
        args.source, args.output, args.first_symbol, args.gain
    )
    flowgraph.run()
    return 0 if report_transport_stream(args.output) else 2


if __name__ == "__main__":
    raise SystemExit(main())
