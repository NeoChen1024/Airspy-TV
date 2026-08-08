#!/usr/bin/env python3

"""Generate a finite DVB-T CS16 regression fixture or stream it to stdout."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from fractions import Fraction
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import BinaryIO

import numpy as np
from gnuradio import blocks, digital, dtv, filter, gr

OUTPUT_SAMPLE_RATE = 10_000_000
CS16_SCALE = 16_384.0
STARTUP_SAMPLES = 20_000
MAX_VECTOR_SAMPLES = 1_000_000

TRANSMISSION_MODES = {
    "2k": (dtv.T2k, 2_048, 1_512),
    "8k": (dtv.T8k, 8_192, 6_048),
}
GUARD_INTERVALS = {
    "1/32": (dtv.GI_1_32, 32),
    "1/16": (dtv.GI_1_16, 16),
    "1/8": (dtv.GI_1_8, 8),
    "1/4": (dtv.GI_1_4, 4),
}
CONSTELLATIONS = {
    "qpsk": dtv.MOD_QPSK,
    "qam16": dtv.MOD_16QAM,
    "qam64": dtv.MOD_64QAM,
}
CODE_RATES = {
    "1/2": dtv.C1_2,
    "2/3": dtv.C2_3,
    "3/4": dtv.C3_4,
    "5/6": dtv.C5_6,
    "7/8": dtv.C7_8,
}
CHANNEL_BANDWIDTHS = {
    "5M": 5_000_000,
    "6M": 6_000_000,
    "7M": 7_000_000,
    "8M": 8_000_000,
}


@dataclass(frozen=True)
class DvbtConfig:
    transmission_mode: str
    channel_bandwidth: str
    guard_interval: str
    constellation: str
    code_rate: str

    @property
    def bandwidth_hz(self) -> int:
        return CHANNEL_BANDWIDTHS[self.channel_bandwidth]


class StreamingCs16Sink(gr.sync_block):
    """Apply independent SRO/CFO impairments and emit bounded CS16 chunks."""

    def __init__(
        self,
        output: BinaryIO,
        output_samples: int,
        sample_clock_ppm: float,
        sample_clock_drift_ppm_per_minute: float,
        lo_offset_hz: float,
        lo_drift_hz_per_minute: float,
    ) -> None:
        super().__init__(
            name="DVB-T streaming CS16 sink",
            in_sig=[np.complex64],
            out_sig=None,
        )
        self._output = output
        self._output_samples = output_samples
        self._sample_clock_ppm = sample_clock_ppm
        self._sample_clock_drift_ppm_per_minute = sample_clock_drift_ppm_per_minute
        self._lo_offset_hz = lo_offset_hz
        self._lo_drift_hz_per_minute = lo_drift_hz_per_minute
        duration_seconds = output_samples / OUTPUT_SAMPLE_RATE
        final_clock_ppm = (
            sample_clock_ppm
            + sample_clock_drift_ppm_per_minute * duration_seconds / 60.0
        )
        self._max_clock_scale = 1.0 + max(sample_clock_ppm, final_clock_ppm) * 1.0e-6

        self._pending = np.empty(0, dtype=np.complex64)
        self._pending_start = 0
        self._source_position = 0.0
        self._emitted = 0
        self._peak = 0.0
        self._power_sum = 0.0
        self._clipped_components = 0
        self._error = ""
        self._done = False

    @property
    def emitted_samples(self) -> int:
        return self._emitted

    @property
    def peak(self) -> float:
        return self._peak

    @property
    def rms_dbfs(self) -> float:
        if self._emitted == 0 or self._power_sum <= 0.0:
            return -math.inf
        rms = math.sqrt(self._power_sum / self._emitted)
        rms *= CS16_SCALE / 32768.0
        return 20.0 * math.log10(rms)

    @property
    def clipped_components(self) -> int:
        return self._clipped_components

    def work(self, input_items, output_items) -> int:  # type: ignore[no-untyped-def]
        del output_items
        if self._done:
            return -1
        incoming = np.asarray(input_items[0], dtype=np.complex64)
        if incoming.size != 0:
            self._pending = np.concatenate((self._pending, incoming.copy()))
        self._emit_available()
        return -1 if self._done else len(incoming)

    def finish(self) -> None:
        try:
            self._output.flush()
        except (BrokenPipeError, OSError) as exception:
            if not self._error:
                self._error = f"unable to flush CS16 output: {exception}"
        if self._error:
            raise RuntimeError(self._error)
        if self._emitted != self._output_samples:
            raise RuntimeError(
                "GNU Radio produced too few samples for streaming impairment: "
                f"{self._emitted} of {self._output_samples}"
            )

    def _emit_available(self) -> None:
        while self._emitted < self._output_samples and self._pending.size >= 2:
            last_source_sample = self._pending_start + self._pending.size - 1
            available_span = last_source_sample - self._source_position
            if available_span <= 0.0:
                return
            estimate = max(1, math.ceil(available_span * self._max_clock_scale) + 2)
            count = min(
                self._output_samples - self._emitted,
                estimate,
                MAX_VECTOR_SAMPLES,
            )
            sample_numbers = np.arange(
                self._emitted,
                self._emitted + count,
                dtype=np.float64,
            )
            time_seconds = sample_numbers / float(OUTPUT_SAMPLE_RATE)
            clock_ppm = (
                self._sample_clock_ppm
                + self._sample_clock_drift_ppm_per_minute * time_seconds / 60.0
            )
            clock_scale = 1.0 + clock_ppm * 1.0e-6
            if np.any(clock_scale <= 0.0):
                self._error = "sample-clock impairment makes the clock non-positive"
                self._done = True
                return
            increments = 1.0 / clock_scale
            positions = np.empty(count, dtype=np.float64)
            positions[0] = self._source_position
            if count > 1:
                positions[1:] = self._source_position + np.cumsum(increments[:-1])
            available_count = int(
                np.searchsorted(positions, last_source_sample, side="left")
            )
            if available_count == 0:
                return

            positions = positions[:available_count]
            increments = increments[:available_count]
            time_seconds = time_seconds[:available_count]
            left_absolute = np.floor(positions).astype(np.int64)
            left = left_absolute - self._pending_start
            fraction = (positions - left_absolute).astype(np.float32)
            impaired = (
                self._pending[left] * (1.0 - fraction)
                + self._pending[left + 1] * fraction
            ).astype(np.complex64)

            if self._lo_offset_hz != 0.0 or self._lo_drift_hz_per_minute != 0.0:
                drift_hz_per_second = self._lo_drift_hz_per_minute / 60.0
                phase = (
                    2.0
                    * np.pi
                    * (
                        self._lo_offset_hz * time_seconds
                        + 0.5 * drift_hz_per_second * time_seconds**2
                    )
                )
                impaired *= np.exp(1j * phase).astype(np.complex64)

            self._write(impaired)
            if self._error:
                self._done = True
                return
            self._source_position = positions[-1] + increments[-1]
            self._emitted += available_count
            discard = max(0, math.floor(self._source_position) - self._pending_start)
            if discard != 0:
                self._pending = self._pending[discard:]
                self._pending_start += discard
            if self._emitted == self._output_samples:
                self._done = True
                return
            if available_count != count:
                return

    def _write(self, samples: np.ndarray) -> None:
        magnitudes = np.abs(samples)
        self._peak = max(self._peak, float(np.max(magnitudes)))
        self._power_sum += float(np.sum(magnitudes * magnitudes, dtype=np.float64))

        real = np.rint(samples.real * CS16_SCALE)
        imag = np.rint(samples.imag * CS16_SCALE)
        self._clipped_components += int(
            np.count_nonzero((real < -32768.0) | (real > 32767.0))
            + np.count_nonzero((imag < -32768.0) | (imag > 32767.0))
        )
        iq = np.empty(samples.size * 2, dtype="<i2")
        iq[0::2] = np.clip(real, -32768.0, 32767.0).astype(np.int16)
        iq[1::2] = np.clip(imag, -32768.0, 32767.0).astype(np.int16)
        payload = iq.tobytes()
        try:
            written = self._output.write(payload)
            if written is not None and written != len(payload):
                self._error = "short write while streaming CS16 output"
        except (BrokenPipeError, OSError) as exception:
            self._error = f"unable to stream CS16 output: {exception}"


class DvbtTransmitter(gr.top_block):
    """GNU Radio DVB-T reference transmitter with a finite streaming sink."""

    def __init__(
        self,
        source: Path,
        sink: StreamingCs16Sink,
        samples: int,
        config: DvbtConfig,
    ) -> None:
        super().__init__("Ideal DVB-T fixture transmitter", catch_exceptions=True)

        transmission_mode, fft_length, payload_carriers = TRANSMISSION_MODES[
            config.transmission_mode
        ]
        guard_interval, guard_divisor = GUARD_INTERVALS[config.guard_interval]
        constellation = CONSTELLATIONS[config.constellation]
        code_rate = CODE_RATES[config.code_rate]
        cp_length = fft_length // guard_divisor
        resampling = Fraction(OUTPUT_SAMPLE_RATE * 7, config.bandwidth_hz * 8)

        source_block = blocks.file_source(gr.sizeof_char, str(source), True)
        energy_dispersal = dtv.dvbt_energy_dispersal(1)
        reed_solomon = dtv.dvbt_reed_solomon_enc(2, 8, 0x11D, 255, 239, 8, 51, 8)
        convolutional_interleaver = dtv.dvbt_convolutional_interleaver(136, 12, 17)
        inner_coder = dtv.dvbt_inner_coder(
            1, payload_carriers, constellation, dtv.NH, code_rate
        )
        bit_interleaver = dtv.dvbt_bit_inner_interleaver(
            payload_carriers,
            constellation,
            dtv.NH,
            transmission_mode,
        )
        symbol_interleaver = dtv.dvbt_symbol_inner_interleaver(
            payload_carriers, transmission_mode, 1
        )
        mapper = dtv.dvbt_map(
            payload_carriers,
            constellation,
            dtv.NH,
            transmission_mode,
            1.0,
        )
        reference_signals = dtv.dvbt_reference_signals(
            gr.sizeof_gr_complex,
            payload_carriers,
            fft_length,
            constellation,
            dtv.NH,
            code_rate,
            code_rate,
            guard_interval,
            transmission_mode,
            1,
            0,
        )
        cyclic_prefix = digital.ofdm_cyclic_prefixer(
            fft_length, fft_length + cp_length, 0, ""
        )
        resampler = filter.rational_resampler_ccc(
            interpolation=resampling.numerator,
            decimation=resampling.denominator,
        )
        skip_startup = blocks.skiphead(gr.sizeof_gr_complex, STARTUP_SAMPLES)
        head = blocks.head(gr.sizeof_gr_complex, samples)

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
        sample_clock_ppm + sample_clock_drift_ppm_per_minute * duration_seconds / 60.0
    )
    minimum_clock_scale = 1.0 + min(sample_clock_ppm, final_ppm) * 1.0e-6
    if minimum_clock_scale <= 0.0:
        raise ValueError("sample-clock impairment makes the clock non-positive")
    return math.ceil(output_samples / minimum_clock_scale) + 8


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("output", help="output .cs16 path, or - for stdout")
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--center-frequency", type=int, default=557_000_000)
    parser.add_argument(
        "--expected-ts",
        type=Path,
        help="retain the unmodulated source MPEG-TS at this path",
    )
    parser.add_argument("--dvbt-mode", choices=TRANSMISSION_MODES, default="8k")
    parser.add_argument(
        "--dvbt-channel-bandwidth",
        choices=CHANNEL_BANDWIDTHS,
        default="6M",
    )
    parser.add_argument("--dvbt-guard", choices=GUARD_INTERVALS, default="1/4")
    parser.add_argument("--dvbt-modulation", choices=CONSTELLATIONS, default="qam64")
    parser.add_argument("--dvbt-code-rate", choices=CODE_RATES, default="2/3")
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
    duration_seconds = args.duration
    final_clock_ppm = (
        args.sample_clock_ppm
        + args.sample_clock_drift_ppm_per_minute * duration_seconds / 60.0
    )
    if 1.0 + min(args.sample_clock_ppm, final_clock_ppm) * 1.0e-6 <= 0.0:
        parser.error("sample-clock impairment makes the clock non-positive")
    return args


def run_generator(
    args: argparse.Namespace,
    output: BinaryIO,
    expected_ts: Path,
    config: DvbtConfig,
) -> StreamingCs16Sink:
    make_transport_stream(expected_ts, max(args.duration, 1.0))
    sample_count = round(args.duration * OUTPUT_SAMPLE_RATE)
    if sample_count <= 0:
        raise ValueError("duration is too short to produce a sample")
    transmitter_sample_count = required_transmitter_samples(
        sample_count,
        args.sample_clock_ppm,
        args.sample_clock_drift_ppm_per_minute,
    )
    sink = StreamingCs16Sink(
        output,
        sample_count,
        args.sample_clock_ppm,
        args.sample_clock_drift_ppm_per_minute,
        args.lo_offset_hz,
        args.lo_drift_hz_per_minute,
    )
    DvbtTransmitter(expected_ts, sink, transmitter_sample_count, config).run()
    sink.finish()
    return sink


def write_metadata(
    sidecar: Path,
    output: Path,
    args: argparse.Namespace,
    config: DvbtConfig,
    sink: StreamingCs16Sink,
) -> None:
    metadata = {
        "data_file": output.name,
        "datatype": "ci16_le",
        "iq_order": "IQ",
        "sample_rate": OUTPUT_SAMPLE_RATE,
        "center_frequency": args.center_frequency,
        "source": "GNU Radio ideal DVB-T fixture",
        "complex_samples": sink.emitted_samples,
        "duration_ms": round(sink.emitted_samples * 1000 / OUTPUT_SAMPLE_RATE),
        "dropped_blocks": 0,
        "source_dropped_samples": 0,
        "dvbt": {
            "transmission_mode": config.transmission_mode,
            "channel_bandwidth_hz": config.bandwidth_hz,
            "guard_interval": config.guard_interval,
            "constellation": config.constellation,
            "code_rate": config.code_rate,
        },
        "impairments": {
            "sample_clock_offset_ppm": args.sample_clock_ppm,
            "sample_clock_drift_ppm_per_minute": (
                args.sample_clock_drift_ppm_per_minute
            ),
            "lo_offset_hz": args.lo_offset_hz,
            "lo_drift_hz_per_minute": args.lo_drift_hz_per_minute,
        },
        "quantization": {
            "cs16_scale": CS16_SCALE,
            "rms_dbfs": sink.rms_dbfs,
            "clipped_components": sink.clipped_components,
        },
    }
    sidecar.write_text(json.dumps(metadata, indent=2) + "\n")


def report_result(
    output_name: str,
    expected_ts: Path,
    args: argparse.Namespace,
    config: DvbtConfig,
    sink: StreamingCs16Sink,
    sidecar: Path | None,
) -> None:
    print(
        f"generated {output_name}: {sink.emitted_samples} samples at "
        f"{OUTPUT_SAMPLE_RATE / 1e6:g} MSPS",
        file=sys.stderr,
    )
    print(
        "DVB-T: "
        f"{config.channel_bandwidth}, {config.transmission_mode.upper()}, "
        f"GI {config.guard_interval}, {config.constellation.upper()}, "
        f"code rate {config.code_rate}; CF32 peak={sink.peak:.6g}, "
        f"CS16 RMS={sink.rms_dbfs:.2f} dBFS, "
        f"clipped components={sink.clipped_components}",
        file=sys.stderr,
    )
    print(
        "impairments: "
        f"sample clock={args.sample_clock_ppm:+g} ppm, "
        f"sample drift={args.sample_clock_drift_ppm_per_minute:+g} "
        "ppm/min, "
        f"LO={args.lo_offset_hz:+g} Hz, "
        f"LO drift={args.lo_drift_hz_per_minute:+g} Hz/min",
        file=sys.stderr,
    )
    print(f"source transport stream: {expected_ts}", file=sys.stderr)
    if sidecar is not None:
        print(f"metadata: {sidecar}", file=sys.stderr)


def main() -> int:
    args = parse_args()
    config = DvbtConfig(
        transmission_mode=args.dvbt_mode,
        channel_bandwidth=args.dvbt_channel_bandwidth,
        guard_interval=args.dvbt_guard,
        constellation=args.dvbt_modulation,
        code_rate=args.dvbt_code_rate,
    )

    if args.output == "-":
        binary_fd = os.dup(sys.stdout.fileno())
        os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
        with os.fdopen(binary_fd, "wb", buffering=0) as binary_output:
            if args.expected_ts is not None:
                expected_ts = args.expected_ts.resolve()
                expected_ts.parent.mkdir(parents=True, exist_ok=True)
                sink = run_generator(args, binary_output, expected_ts, config)
                report_result("stdout", expected_ts, args, config, sink, None)
            else:
                with tempfile.TemporaryDirectory(
                    prefix="airspy-tv-fixture-"
                ) as temporary:
                    expected_ts = Path(temporary) / "source.expected.ts"
                    sink = run_generator(args, binary_output, expected_ts, config)
                    report_result("stdout", expected_ts, args, config, sink, None)
        return 0

    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    expected_ts = (
        args.expected_ts.resolve()
        if args.expected_ts is not None
        else output.with_suffix(".expected.ts")
    )
    expected_ts.parent.mkdir(parents=True, exist_ok=True)
    sidecar = Path(str(output) + ".json")
    with output.open("wb") as destination:
        sink = run_generator(args, destination, expected_ts, config)
    write_metadata(sidecar, output, args, config, sink)
    report_result(str(output), expected_ts, args, config, sink, sidecar)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
