#!/usr/bin/env python3

"""Experimental soft-decision DVB-T decoder for equalized 8K/64-QAM data."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from gnuradio import blocks, dtv, fec, gr

from dvbt_extract import PAYLOAD_CARRIERS, report_transport_stream


BITS_PER_SYMBOL = 6
BIT_INTERLEAVER_SIZE = 126
SYMBOLS_PER_FRAME = 68
VITERBI_FRAME_BITS = 768
PUNCTURE_2_3 = np.asarray([1, 1, 0, 1], dtype=np.uint8)


def constellation_64qam() -> np.ndarray:
    points = np.empty(64, dtype=np.complex64)
    norm = 1.0 / np.sqrt(42.0)
    for source in range(64):
        quadrant = (source >> 4) & 3
        sign_i = -1 if quadrant & 2 else 1
        sign_q = -1 if quadrant & 1 else 1
        axis_i = (source >> 2) & 3
        axis_q = source & 3
        value_i = 1 + (3 - axis_i) * 2
        value_q = 1 + (3 - axis_q) * 2
        gray = ((axis_i ^ (axis_i >> 1)) << 2) + (
            axis_q ^ (axis_q >> 1)
        )
        bits_i = ((gray >> 1) & 1) | (((gray >> 3) & 1) << 1)
        bits_q = (gray & 1) | (((gray >> 2) & 1) << 1)
        index = (quadrant << 4) + (bits_i << 2) + bits_q
        points[index] = norm * complex(sign_i * value_i, sign_q * value_q)
    return points


def symbol_permutation() -> np.ndarray:
    bit_permutation = (7, 1, 4, 2, 9, 6, 8, 10, 0, 3, 11, 5)
    accepted: list[int] = []
    register = 0
    for index in range(8192):
        if index < 2:
            register = 0
        elif index == 2:
            register = 1
        else:
            new_bit = (
                register
                ^ (register >> 1)
                ^ (register >> 4)
                ^ (register >> 6)
            ) & 1
            register = ((register >> 1) | (new_bit << 11)) & 0x1FFF
        permuted = 0
        for bit, destination in enumerate(bit_permutation):
            permuted |= ((register >> bit) & 1) << destination
        value = ((index & 1) << 12) + permuted
        if value < PAYLOAD_CARRIERS:
            accepted.append(value)
    if len(accepted) != PAYLOAD_CARRIERS:
        raise RuntimeError(f"invalid H(q) table: {len(accepted)} entries")
    return np.asarray(accepted, dtype=np.intp)


def max_log_metrics(
    samples: np.ndarray, points: np.ndarray, point_bits: np.ndarray
) -> np.ndarray:
    """Return unsigned metrics: 0 is a confident zero, 255 a confident one."""
    # Correct the common amplitude/phase error left on each OFDM symbol. Two
    # decision-directed iterations are sufficient and preserve per-carrier noise.
    corrected = samples.astype(np.complex64, copy=True)
    for _ in range(2):
        distances = np.abs(corrected[:, None] - points[None, :]) ** 2
        decisions = points[np.argmin(distances, axis=1)]
        gain = np.vdot(decisions, corrected) / np.vdot(decisions, decisions)
        if abs(gain) > 1e-6:
            corrected /= gain

    distances = np.abs(corrected[:, None] - points[None, :]) ** 2
    nearest = np.min(distances, axis=1)
    noise_variance = max(float(np.median(nearest) / np.log(2.0)), 1e-4)
    metrics = np.empty((len(samples), BITS_PER_SYMBOL), dtype=np.uint8)
    for bit in range(BITS_PER_SYMBOL):
        minimum_zero = np.min(distances[:, point_bits[:, bit] == 0], axis=1)
        minimum_one = np.min(distances[:, point_bits[:, bit] == 1], axis=1)
        signed = (minimum_zero - minimum_one) / (2.0 * noise_variance)
        metrics[:, bit] = np.rint(
            127.5 + 127.5 * np.tanh(signed)
        ).astype(np.uint8)
    return metrics


def deinterleave_symbol(metrics: np.ndarray, table: np.ndarray, parity: int):
    output = np.empty_like(metrics)
    if parity:
        output[table] = metrics
    else:
        output = metrics[table]
    return output


def deinterleave_bits(metrics: np.ndarray) -> np.ndarray:
    blocks = metrics.reshape(-1, BIT_INTERLEAVER_SIZE, BITS_PER_SYMBOL)
    output = np.empty_like(blocks)
    offsets = np.asarray([0, 63, 105, 42, 21, 84])
    permutation = np.asarray([0, 2, 4, 1, 3, 5])
    positions = np.arange(BIT_INTERLEAVER_SIZE)
    for output_bit, source_bit in enumerate(permutation):
        source_positions = (positions - offsets[source_bit]) % 126
        output[:, :, output_bit] = blocks[:, source_positions, source_bit]
    return output.reshape(-1)


def depuncture_2_3(metrics: np.ndarray) -> np.ndarray:
    output_length = metrics.size * len(PUNCTURE_2_3) // int(PUNCTURE_2_3.sum())
    output = np.full(output_length, 128, dtype=np.uint8)
    transmitted = np.resize(PUNCTURE_2_3.astype(bool), output_length)
    if int(transmitted.sum()) != metrics.size:
        raise RuntimeError("soft-bit stream is not aligned to the 2/3 puncture period")
    output[transmitted] = metrics
    return output


def write_soft_metrics(source: Path, destination: Path, first_symbol: int) -> int:
    raw = np.memmap(source, dtype=np.complex64, mode="r")
    symbol_count = raw.size // PAYLOAD_CARRIERS
    # Two OFDM symbols produce an integral number of 768-bit Viterbi frames.
    symbol_count -= symbol_count % 2
    symbols = raw[: symbol_count * PAYLOAD_CARRIERS].reshape(
        symbol_count, PAYLOAD_CARRIERS
    )
    points = constellation_64qam()
    point_bits = (
        (np.arange(64)[:, None] >> np.arange(5, -1, -1)) & 1
    ).astype(np.uint8)
    table = symbol_permutation()

    with destination.open("wb") as stream:
        for symbol_index, carriers in enumerate(symbols):
            soft = max_log_metrics(carriers, points, point_bits)
            soft = deinterleave_symbol(
                soft, table, (first_symbol + symbol_index) % 2
            )
            soft = depuncture_2_3(deinterleave_bits(soft))
            soft.tofile(stream)
            if (symbol_index + 1) % 250 == 0:
                print(f"soft demap: {symbol_index + 1}/{symbol_count} symbols")
    return symbol_count


class SoftFecDecoder(gr.top_block):
    def __init__(
        self,
        metrics: Path,
        output: Path,
        polynomials: list[int],
        pre_rs_output: Path | None,
    ):
        super().__init__("DVB-T soft FEC decoder", catch_exceptions=True)
        source = blocks.file_source(gr.sizeof_char, str(metrics), False)
        decoder_object = fec.cc_decoder.make(
            VITERBI_FRAME_BITS,
            7,
            2,
            polynomials,
            0,
            -1,
            fec.CC_STREAMING,
            False,
        )
        viterbi = fec.decoder(decoder_object, 1, 1)
        pack = blocks.unpacked_to_packed_bb(1, gr.GR_MSB_FIRST)
        byte_deinterleaver = dtv.dvbt_convolutional_deinterleaver(136, 12, 17)
        reed_solomon = dtv.dvbt_reed_solomon_dec(
            2, 8, 0x11D, 255, 239, 8, 51, 8
        )
        descrambler = dtv.dvbt_energy_descramble(8)
        sink = blocks.file_sink(gr.sizeof_char, str(output), False)
        if pre_rs_output is not None:
            pre_rs_sink = blocks.file_sink(
                gr.sizeof_char, str(pre_rs_output), False
            )
            pre_rs_stream = blocks.vector_to_stream(gr.sizeof_char, 1632)
            self.connect(byte_deinterleaver, pre_rs_stream, pre_rs_sink)
        self.connect(
            source,
            viterbi,
            pack,
            byte_deinterleaver,
            reed_solomon,
            descrambler,
            sink,
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path, help="6048-carrier complex64 file")
    parser.add_argument("output", type=Path)
    parser.add_argument("--metrics", type=Path, default=Path("/tmp/dvbt-soft.u8"))
    parser.add_argument("--first-symbol", type=int, default=0)
    parser.add_argument(
        "--polynomial-order", choices=("gr-fec", "gr-dtv"), default="gr-dtv"
    )
    parser.add_argument("--reuse-metrics", action="store_true")
    parser.add_argument("--pre-rs-output", type=Path)
    args = parser.parse_args()

    if not args.reuse_metrics:
        count = write_soft_metrics(args.source, args.metrics, args.first_symbol)
        print(f"soft metrics: {count} OFDM symbols -> {args.metrics}")
    polynomials = [109, 79] if args.polynomial_order == "gr-fec" else [79, 109]
    flowgraph = SoftFecDecoder(
        args.metrics, args.output, polynomials, args.pre_rs_output
    )
    flowgraph.run()
    return 0 if report_transport_stream(args.output) else 2


if __name__ == "__main__":
    raise SystemExit(main())
