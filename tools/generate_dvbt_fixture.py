#!/usr/bin/env python3

"""Generate an ideal centered 6 MHz DVB-T CS16 regression fixture."""

from __future__ import annotations

import argparse
from collections.abc import Iterator
import json
import math
from pathlib import Path
import subprocess

import numpy as np
from gnuradio import blocks, digital, dtv, filter, gr


NATIVE_SAMPLE_RATE = 48_000_000 / 7
OUTPUT_SAMPLE_RATE = 10_000_000
FFT_LENGTH = 8192
PAYLOAD_CARRIERS = 6048
CP_LENGTH = FFT_LENGTH // 4


class DvbtTransmitter(gr.top_block):
    """GNU Radio DVB-T reference transmitter with a finite CF32 output."""

    def __init__(self, source: Path, output: Path, samples: int) -> None:
        super().__init__("Ideal DVB-T fixture transmitter", catch_exceptions=True)

        constellation = dtv.MOD_64QAM
        hierarchy = dtv.NH
        code_rate = dtv.C2_3
        transmission_mode = dtv.T8k

        source_block = blocks.file_source(
            gr.sizeof_char, str(source), True
        )
        energy_dispersal = dtv.dvbt_energy_dispersal(1)
        reed_solomon = dtv.dvbt_reed_solomon_enc(
            2, 8, 0x11D, 255, 239, 8, 51, 8
        )
        convolutional_interleaver = dtv.dvbt_convolutional_interleaver(
            136, 12, 17
        )
        inner_coder = dtv.dvbt_inner_coder(
            1, PAYLOAD_CARRIERS, constellation, hierarchy, code_rate
        )
        bit_interleaver = dtv.dvbt_bit_inner_interleaver(
            PAYLOAD_CARRIERS,
            constellation,
            hierarchy,
            transmission_mode,
        )
        symbol_interleaver = dtv.dvbt_symbol_inner_interleaver(
            PAYLOAD_CARRIERS, transmission_mode, 1
        )
        mapper = dtv.dvbt_map(
            PAYLOAD_CARRIERS,
            constellation,
            hierarchy,
            transmission_mode,
            1.0,
        )
        reference_signals = dtv.dvbt_reference_signals(
            gr.sizeof_gr_complex,
            PAYLOAD_CARRIERS,
            FFT_LENGTH,
            constellation,
            hierarchy,
            code_rate,
            code_rate,
            dtv.GI_1_4,
            transmission_mode,
            1,
            0,
        )
        cyclic_prefix = digital.ofdm_cyclic_prefixer(
            FFT_LENGTH, FFT_LENGTH + CP_LENGTH, 0, ""
        )
        resampler = filter.rational_resampler_ccc(
            interpolation=35, decimation=24
        )
        # Discard the scheduler/filter startup transient so one anomalous
        # sample does not determine the CS16 full-scale normalization.
        skip_startup = blocks.skiphead(gr.sizeof_gr_complex, 20_000)
        head = blocks.head(gr.sizeof_gr_complex, samples)
        sink = blocks.file_sink(gr.sizeof_gr_complex, str(output), False)

        self.connect(
            source_block,
            energy_dispersal,
            reed_solomon,
            convolutional_interleaver,
            inner_coder,
            bit_interleaver,
            symbol_interleaver,
            mapper,
            reference_signals,
            cyclic_prefix,
            resampler,
            skip_startup,
            head,
            sink,
        )


def make_transport_stream(path: Path, duration: float) -> None:
    """Create a small real MPEG-TS carrying PAT, PMT, video, and audio."""

    command = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-f",
        "lavfi",
        "-i",
        "testsrc2=size=320x180:rate=25",
        "-f",
        "lavfi",
        "-i",
        "sine=frequency=1000:sample_rate=48000",
        "-t",
        str(duration),
        "-c:v",
        "mpeg2video",
        "-b:v",
        "1500k",
        "-g",
        "12",
        "-c:a",
        "mp2",
        "-b:a",
        "128k",
        "-muxrate",
        "4000k",
        "-mpegts_service_id",
        "1",
        "-mpegts_pmt_start_pid",
        "4096",
        "-mpegts_start_pid",
        "256",
        "-metadata",
        "service_name=Airspy TV fixture",
        "-f",
        "mpegts",
        str(path),
    ]
    subprocess.run(command, check=True)
    if path.stat().st_size % 188 != 0:
        raise RuntimeError("ffmpeg output is not packet-aligned MPEG-TS")


def required_transmitter_samples(
    output_samples: int,
    sample_clock_ppm: float,
    sample_clock_drift_ppm_per_minute: float,
) -> int:
    """Return enough ideal samples to cover the receiver-clock time warp."""

    duration_seconds = output_samples / OUTPUT_SAMPLE_RATE
    final_ppm = (
        sample_clock_ppm
        + sample_clock_drift_ppm_per_minute * duration_seconds / 60.0
    )
    minimum_clock_scale = 1.0 + min(sample_clock_ppm, final_ppm) * 1.0e-6
    if minimum_clock_scale <= 0.0:
        raise ValueError("sample-clock impairment makes the clock non-positive")
    return math.ceil(output_samples / minimum_clock_scale) + 4


def impair_and_quantize_cf32(
    source: Path,
    output: Path,
    output_samples: int,
    sample_clock_ppm: float,
    sample_clock_drift_ppm_per_minute: float,
    lo_offset_hz: float,
    lo_drift_hz_per_minute: float,
) -> tuple[int, float, float]:
    ideal = np.memmap(source, dtype=np.complex64, mode="r")
    if ideal.size == 0:
        raise RuntimeError("GNU Radio produced no baseband samples")

    def impaired_chunks() -> Iterator[np.ndarray]:
        source_position = 0.0
        chunk_samples = 1_000_000
        for start in range(0, output_samples, chunk_samples):
            stop = min(start + chunk_samples, output_samples)
            count = stop - start
            time_seconds = np.arange(start, stop, dtype=np.float64) / float(
                OUTPUT_SAMPLE_RATE
            )
            if (
                sample_clock_ppm != 0.0
                or sample_clock_drift_ppm_per_minute != 0.0
            ):
                clock_ppm = (
                    sample_clock_ppm
                    + sample_clock_drift_ppm_per_minute
                    * time_seconds
                    / 60.0
                )
                clock_scale = 1.0 + clock_ppm * 1.0e-6
                if np.any(clock_scale <= 0.0):
                    raise RuntimeError(
                        "sample-clock impairment makes the clock non-positive"
                    )
                increments = 1.0 / clock_scale
                source_positions = np.empty(count, dtype=np.float64)
                source_positions[0] = source_position
                if count > 1:
                    source_positions[1:] = source_position + np.cumsum(
                        increments[:-1]
                    )
                source_position = source_positions[-1] + increments[-1]
                if source_positions[-1] >= ideal.size - 1:
                    raise RuntimeError(
                        "not enough ideal samples for clock impairment"
                    )
                left = np.floor(source_positions).astype(np.int64)
                fraction = (source_positions - left).astype(np.float32)
                samples = (
                    ideal[left] * (1.0 - fraction)
                    + ideal[left + 1] * fraction
                ).astype(np.complex64)
            else:
                if stop > ideal.size:
                    raise RuntimeError(
                        "GNU Radio produced fewer samples than requested"
                    )
                samples = np.asarray(ideal[start:stop]).copy()

            if lo_offset_hz != 0.0 or lo_drift_hz_per_minute != 0.0:
                drift_hz_per_second = lo_drift_hz_per_minute / 60.0
                phase = 2.0 * np.pi * (
                    lo_offset_hz * time_seconds
                    + 0.5 * drift_hz_per_second * time_seconds**2
                )
                samples *= np.exp(1j * phase).astype(np.complex64)
            yield samples

    peak = 0.0
    power_sum = 0.0
    for samples in impaired_chunks():
        peak = max(peak, float(np.max(np.abs(samples))))
        power_sum += float(np.sum(np.abs(samples) ** 2, dtype=np.float64))
    if not np.isfinite(peak) or peak <= 0.0:
        raise RuntimeError("GNU Radio produced an invalid waveform")
    scale = 0.85 * 32767.0 / peak
    with output.open("wb") as sink:
        for samples in impaired_chunks():
            iq = np.empty(samples.size * 2, dtype=np.int16)
            iq[0::2] = np.rint(samples.real * scale).astype(np.int16)
            iq[1::2] = np.rint(samples.imag * scale).astype(np.int16)
            iq.tofile(sink)

    rms = math.sqrt(power_sum / output_samples) * scale / 32768.0
    rms_dbfs = 20.0 * math.log10(rms)
    return output_samples, peak, rms_dbfs


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path, help="output .cs16 path")
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--center-frequency", type=int, default=557_000_000)
    parser.add_argument(
        "--sample-clock-ppm",
        type=float,
        default=0.0,
        help=(
            "initial receiver sample-clock offset in ppm; positive values "
            "sample the ideal waveform more slowly per recorded sample"
        ),
    )
    parser.add_argument(
        "--sample-clock-drift-ppm-per-minute",
        type=float,
        default=0.0,
        help="linear change of sample-clock offset in ppm per minute",
    )
    parser.add_argument(
        "--lo-offset-hz",
        type=float,
        default=0.0,
        help="initial baseband LO offset in Hz; positive rotates toward +f",
    )
    parser.add_argument(
        "--lo-drift-hz-per-minute",
        type=float,
        default=0.0,
        help="linear change of LO offset in Hz per minute",
    )
    args = parser.parse_args()
    if args.duration <= 0.0:
        parser.error("duration must be positive")
    impairment_values = (
        args.sample_clock_ppm,
        args.sample_clock_drift_ppm_per_minute,
        args.lo_offset_hz,
        args.lo_drift_hz_per_minute,
    )
    if not all(math.isfinite(value) for value in impairment_values):
        parser.error("clock impairment values must be finite")

    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    expected_ts = output.with_suffix(".expected.ts")
    temporary_cf32 = output.with_suffix(".cf32.tmp")
    sidecar = Path(str(output) + ".json")

    make_transport_stream(expected_ts, max(args.duration, 1.0))
    sample_count = round(args.duration * OUTPUT_SAMPLE_RATE)
    if sample_count <= 0:
        parser.error("duration is too short to produce a sample")
    transmitter_sample_count = required_transmitter_samples(
        sample_count,
        args.sample_clock_ppm,
        args.sample_clock_drift_ppm_per_minute,
    )
    DvbtTransmitter(
        expected_ts, temporary_cf32, transmitter_sample_count
    ).run()
    try:
        actual_samples, peak, rms_dbfs = impair_and_quantize_cf32(
            temporary_cf32,
            output,
            sample_count,
            args.sample_clock_ppm,
            args.sample_clock_drift_ppm_per_minute,
            args.lo_offset_hz,
            args.lo_drift_hz_per_minute,
        )
    finally:
        temporary_cf32.unlink(missing_ok=True)

    metadata = {
        "data_file": output.name,
        "datatype": "ci16_le",
        "iq_order": "IQ",
        "sample_rate": OUTPUT_SAMPLE_RATE,
        "center_frequency": args.center_frequency,
        "source": "GNU Radio ideal DVB-T fixture",
        "complex_samples": actual_samples,
        "duration_ms": round(actual_samples * 1000 / OUTPUT_SAMPLE_RATE),
        "dropped_blocks": 0,
        "source_dropped_samples": 0,
        "impairments": {
            "sample_clock_offset_ppm": args.sample_clock_ppm,
            "sample_clock_drift_ppm_per_minute": (
                args.sample_clock_drift_ppm_per_minute
            ),
            "lo_offset_hz": args.lo_offset_hz,
            "lo_drift_hz_per_minute": args.lo_drift_hz_per_minute,
        },
    }
    sidecar.write_text(json.dumps(metadata, indent=2) + "\n")

    print(
        f"generated {output}: {actual_samples} samples at "
        f"{OUTPUT_SAMPLE_RATE / 1e6:g} MSPS"
    )
    print(
        "DVB-T: 6 MHz, 8K, GI 1/4, 64-QAM, code rate 2/3; "
        f"CF32 peak before scaling={peak:.6g}, CS16 RMS={rms_dbfs:.2f} dBFS"
    )
    print(
        "impairments: "
        f"sample clock={args.sample_clock_ppm:+g} ppm, "
        f"sample drift={args.sample_clock_drift_ppm_per_minute:+g} "
        "ppm/min, "
        f"LO={args.lo_offset_hz:+g} Hz, "
        f"LO drift={args.lo_drift_hz_per_minute:+g} Hz/min"
    )
    print(f"source transport stream: {expected_ts}")
    print(f"metadata: {sidecar}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
