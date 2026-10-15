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
ACTIVE_CARRIERS = {"2k": 1_705, "8k": 6_817}
FRACTIONAL_DELAY_TAPS = 32
NOISE_POWER_ESTIMATE_SAMPLES = 65_536

# ETSI EN 300 744 Annex B echo set shared by the fixed Ricean (F1) and
# Rayleigh (P1) channels: (rho_i, tau_i in microseconds, theta_i in radians).
ANNEX_B_ECHOES = (
    (0.057662, 1.003019, 4.855121),
    (0.176809, 5.422091, 3.419109),
    (0.407163, 0.518650, 5.864470),
    (0.303585, 2.751772, 2.215894),
    (0.258782, 0.602895, 3.758058),
    (0.061831, 1.016585, 5.430202),
    (0.150340, 0.143556, 3.952093),
    (0.051534, 0.153832, 1.093586),
    (0.185074, 3.324866, 5.775198),
    (0.400967, 1.935570, 0.154459),
    (0.295723, 0.429948, 5.928383),
    (0.350825, 3.228872, 3.053023),
    (0.262909, 0.848831, 0.628578),
    (0.225894, 0.073883, 2.128544),
    (0.170996, 0.203952, 1.099463),
    (0.149723, 0.194207, 3.462951),
    (0.240140, 0.924450, 3.664773),
    (0.116587, 1.381320, 2.833799),
    (0.221155, 0.640512, 3.334290),
    (0.259730, 1.368671, 0.393889),
)
ANNEX_B_RICEAN_K = 10.0
BUILTIN_CHANNEL_PROFILES = ("ricean", "rayleigh")
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


@dataclass(frozen=True)
class PpmProfile:
    """Piecewise-linear ppm values with an exact continuous-time integral."""

    knots: tuple[tuple[float, float], ...]

    def values(self, seconds: np.ndarray) -> np.ndarray:
        times = np.asarray([knot[0] for knot in self.knots], dtype=np.float64)
        values = np.asarray([knot[1] for knot in self.knots], dtype=np.float64)
        return np.interp(seconds, times, values)

    def integral(self, seconds: np.ndarray) -> np.ndarray:
        """Integrate ppm-seconds from t=0 without phase discontinuities."""

        result = np.zeros_like(seconds, dtype=np.float64)
        for index, (begin, begin_value) in enumerate(self.knots):
            end = (
                self.knots[index + 1][0]
                if index + 1 < len(self.knots)
                else math.inf
            )
            end_value = (
                self.knots[index + 1][1]
                if index + 1 < len(self.knots)
                else begin_value
            )
            span = np.clip(seconds - begin, 0.0, end - begin)
            if math.isfinite(end):
                slope = (end_value - begin_value) / (end - begin)
                result += begin_value * span + 0.5 * slope * span * span
            else:
                result += begin_value * span
        return result

    @property
    def minimum(self) -> float:
        return min(value for _, value in self.knots)


def parse_ppm_knots(
    parser: argparse.ArgumentParser,
    values: list[str] | None,
    option: str,
    duration: float,
) -> PpmProfile | None:
    if not values:
        return None
    knots: list[tuple[float, float]] = []
    for value in values:
        try:
            seconds_text, ppm_text = value.split(":", 1)
            knot = (float(seconds_text), float(ppm_text))
        except (ValueError, TypeError):
            parser.error(f"{option} must use SECONDS:PPM")
        if not all(math.isfinite(component) for component in knot):
            parser.error(f"{option} values must be finite")
        knots.append(knot)
    if knots[0][0] != 0.0:
        parser.error(f"the first {option} must start at 0 seconds")
    if any(seconds < 0.0 or seconds > duration for seconds, _ in knots):
        parser.error(f"{option} times must be within the fixture duration")
    if any(right[0] <= left[0] for left, right in zip(knots, knots[1:])):
        parser.error(f"{option} times must be strictly increasing")
    return PpmProfile(tuple(knots))


@dataclass(frozen=True)
class ChannelTap:
    delay_us: float
    gain: complex
    doppler_hz: float

    @property
    def power_db(self) -> float:
        return 20.0 * math.log10(abs(self.gain))

    @property
    def phase_deg(self) -> float:
        return math.degrees(math.atan2(self.gain.imag, self.gain.real))


def normalize_taps(taps: list[ChannelTap]) -> list[ChannelTap]:
    """Scale the tap set to unit total power so the mean signal power, and
    therefore the --snr-db reference, is unchanged by the channel."""
    total = sum(abs(tap.gain) ** 2 for tap in taps)
    if total <= 0.0:
        raise ValueError("channel taps carry no power")
    scale = 1.0 / math.sqrt(total)
    return [
        ChannelTap(tap.delay_us, tap.gain * scale, tap.doppler_hz) for tap in taps
    ]


def builtin_channel_profile(name: str) -> list[ChannelTap]:
    """EN 300 744 Annex B: y(t) = (rho_0 x(t) + sum rho_i e^{-j theta_i}
    x(t - tau_i)) / sqrt(sum rho_i^2); rho_0 = sqrt(K sum rho_i^2) for the
    Ricean channel (K = 10) and 0 for the Rayleigh channel."""
    taps = [
        ChannelTap(tau, rho * complex(math.cos(-theta), math.sin(-theta)), 0.0)
        for rho, tau, theta in ANNEX_B_ECHOES
    ]
    if name == "ricean":
        direct = math.sqrt(ANNEX_B_RICEAN_K * sum(rho**2 for rho, _, _ in ANNEX_B_ECHOES))
        taps.insert(0, ChannelTap(0.0, complex(direct, 0.0), 0.0))
    elif name != "rayleigh":
        raise ValueError(f"unknown built-in channel profile: {name}")
    return normalize_taps(taps)


def load_channel_profile(
    spec: str, min_presence: float, phase_seed: int
) -> tuple[str, list[ChannelTap]]:
    """`ricean`, `rayleigh`, or FILE[:NAME] in the airspy-tv-channel-profile/0
    format written by tools/summarize_channel_profile.py. Measured taps carry
    no phase, so each gets a seeded pseudo-random one."""
    if spec in BUILTIN_CHANNEL_PROFILES:
        return spec, builtin_channel_profile(spec)
    path_text, _, name = spec.partition(":")
    path = Path(path_text)
    profiles = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(profiles, dict):
        profiles = [profiles]
    if name:
        matches = [profile for profile in profiles if profile.get("name") == name]
        if not matches:
            raise ValueError(f"no channel profile named {name!r} in {path}")
        profile = matches[0]
    else:
        profile = profiles[0]
    if profile.get("format") != "airspy-tv-channel-profile/0":
        raise ValueError(f"{path}: unsupported channel profile format")
    rng = np.random.default_rng(phase_seed)
    taps = []
    for entry in profile["taps"]:
        phase = float(rng.uniform(0.0, 2.0 * math.pi))
        if entry.get("presence", 1.0) < min_presence:
            continue
        if "phase_deg" in entry:
            phase = math.radians(float(entry["phase_deg"]))
        amplitude = 10.0 ** (float(entry["power_db"]) / 20.0)
        taps.append(
            ChannelTap(
                float(entry["delay_us"]),
                amplitude * complex(math.cos(phase), math.sin(phase)),
                float(entry.get("doppler_hz") or 0.0),
            )
        )
    if not taps:
        raise ValueError(f"{path}: profile {profile.get('name')!r} has no taps")
    return f"{path.name}:{profile.get('name')}", normalize_taps(taps)


def parse_channel_tap(text: str) -> ChannelTap:
    """DELAY_US:POWER_DB[:PHASE_DEG[:DOPPLER_HZ]]"""
    parts = text.split(":")
    if len(parts) < 2 or len(parts) > 4:
        raise ValueError(f"invalid --channel-tap {text!r}")
    delay_us = float(parts[0])
    amplitude = 10.0 ** (float(parts[1]) / 20.0)
    phase = math.radians(float(parts[2])) if len(parts) > 2 else 0.0
    doppler_hz = float(parts[3]) if len(parts) > 3 else 0.0
    if not all(math.isfinite(value) for value in (delay_us, amplitude, phase, doppler_hz)):
        raise ValueError(f"non-finite --channel-tap {text!r}")
    return ChannelTap(delay_us, amplitude * complex(math.cos(phase), math.sin(phase)), doppler_hz)


class MultipathChannel:
    """Static multipath (plus optional per-tap Doppler) at the transmitter
    sample rate. Every tap is a Kaiser-windowed-sinc fractional delay; taps
    without Doppler are merged into one FIR applied by FFT convolution, the
    others are filtered separately and rotated. The common latency (half the
    sinc length plus the shift that makes the earliest tap causal) applies to
    every tap and is irrelevant to the receiver."""

    def __init__(self, taps: list[ChannelTap], sample_rate_hz: float) -> None:
        self._sample_rate_hz = sample_rate_hz
        half = FRACTIONAL_DELAY_TAPS // 2
        window = np.kaiser(FRACTIONAL_DELAY_TAPS, 8.0)
        static_length = 0
        kernels: list[tuple[np.ndarray, int, float]] = []
        # A pre-echo (negative delay relative to the reference path) is
        # realized by delaying every tap so the earliest one sits at zero.
        earliest_us = min(tap.delay_us for tap in taps)
        for tap in taps:
            delay_samples = (tap.delay_us - earliest_us) * 1e-6 * sample_rate_hz
            integer = int(math.floor(delay_samples))
            fraction = delay_samples - integer
            kernel = np.sinc(np.arange(FRACTIONAL_DELAY_TAPS) - half - fraction) * window
            kernel = (kernel / np.sum(kernel)).astype(np.complex128) * tap.gain
            kernels.append((kernel, integer, tap.doppler_hz))
            if tap.doppler_hz == 0.0:
                static_length = max(static_length, integer + FRACTIONAL_DELAY_TAPS)
        self._static = np.zeros(static_length, dtype=np.complex128)
        self._moving: list[tuple[np.ndarray, float]] = []
        for kernel, integer, doppler_hz in kernels:
            if doppler_hz == 0.0:
                self._static[integer : integer + FRACTIONAL_DELAY_TAPS] += kernel
            else:
                shifted = np.zeros(integer + FRACTIONAL_DELAY_TAPS, dtype=np.complex128)
                shifted[integer:] = kernel
                self._moving.append((shifted, doppler_hz))
        self._history_length = max(
            [self._static.size] + [kernel.size for kernel, _ in self._moving]
        ) - 1
        self._history = np.zeros(self._history_length, dtype=np.complex64)
        self._position = 0

    @staticmethod
    def _convolve_valid(signal: np.ndarray, kernel: np.ndarray) -> np.ndarray:
        """Convolution restricted to outputs with full kernel support."""
        if kernel.size == 0:
            return np.zeros(signal.size, dtype=np.complex128)
        size = 1 << (signal.size + kernel.size - 1).bit_length()
        product = np.fft.fft(signal, size) * np.fft.fft(kernel, size)
        full = np.fft.ifft(product)[: signal.size + kernel.size - 1]
        return full[kernel.size - 1 : signal.size]

    def process(self, block: np.ndarray) -> np.ndarray:
        if block.size == 0:
            return block
        signal = np.concatenate((self._history, block)).astype(np.complex128)
        output = np.zeros(block.size, dtype=np.complex128)
        if self._static.size != 0:
            output += self._convolve_valid(signal, self._static)[-block.size :]
        if self._moving:
            time_seconds = (
                np.arange(self._position, self._position + block.size, dtype=np.float64)
                / self._sample_rate_hz
            )
            for kernel, doppler_hz in self._moving:
                path = self._convolve_valid(signal, kernel)[-block.size :]
                output += path * np.exp(2j * np.pi * doppler_hz * time_seconds)
        self._position += block.size
        if self._history_length != 0:
            self._history = np.concatenate((self._history, block))[-self._history_length :]
        return output.astype(np.complex64)


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
        sample_clock_profile: PpmProfile | None = None,
        lo_ppm_profile: PpmProfile | None = None,
        center_frequency_hz: float = 0.0,
        channel: MultipathChannel | None = None,
        snr_db: float | None = None,
        noise_bandwidth_hz: float = 0.0,
        noise_seed: int = 1,
    ) -> None:
        super().__init__(
            name="DVB-T streaming CS16 sink",
            in_sig=[np.complex64],
            out_sig=None,
        )
        self._output = output
        self._channel = channel
        self._snr_db = snr_db
        self._noise_bandwidth_hz = noise_bandwidth_hz
        self._noise_rng = np.random.default_rng(noise_seed)
        self._noise_sigma: float | None = None
        self._output_samples = output_samples
        self._sample_clock_ppm = sample_clock_ppm
        self._sample_clock_drift_ppm_per_minute = sample_clock_drift_ppm_per_minute
        self._lo_offset_hz = lo_offset_hz
        self._lo_drift_hz_per_minute = lo_drift_hz_per_minute
        self._sample_clock_profile = sample_clock_profile
        self._lo_ppm_profile = lo_ppm_profile
        self._center_frequency_hz = center_frequency_hz
        duration_seconds = output_samples / OUTPUT_SAMPLE_RATE
        final_clock_ppm = (
            sample_clock_ppm
            + sample_clock_drift_ppm_per_minute * duration_seconds / 60.0
        )
        maximum_clock_ppm = (
            max(value for _, value in sample_clock_profile.knots)
            if sample_clock_profile is not None
            else max(sample_clock_ppm, final_clock_ppm)
        )
        self._max_clock_scale = 1.0 + maximum_clock_ppm * 1.0e-6

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
            if self._channel is not None:
                incoming = self._channel.process(incoming)
            self._pending = np.concatenate((self._pending, incoming.copy()))
        self._emit_available()
        return -1 if self._done else len(incoming)

    def finish(self) -> None:
        if self._snr_db is not None and self._noise_sigma is None:
            self._noise_sigma = 0.0
            self._noise_ready()
            if self._pending.size != 0:
                self._noise_sigma = None
                reference = self._pending.astype(np.complex128)
                signal_power = float(np.mean(np.abs(reference) ** 2))
                noise_power = (
                    signal_power
                    / (10.0 ** (self._snr_db / 10.0))
                    * (OUTPUT_SAMPLE_RATE / self._noise_bandwidth_hz)
                )
                self._noise_sigma = math.sqrt(noise_power / 2.0)
                self._emit_available()
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

    def _noise_ready(self) -> bool:
        """The AWGN level follows the mean post-channel signal power, measured
        once over the first NOISE_POWER_ESTIMATE_SAMPLES samples. --snr-db is
        C/N inside the occupied signal bandwidth; the noise is white across
        the whole output rate, so its total power is scaled up accordingly."""
        if self._snr_db is None or self._noise_sigma is not None:
            return True
        if self._pending.size < NOISE_POWER_ESTIMATE_SAMPLES:
            return False
        reference = self._pending[:NOISE_POWER_ESTIMATE_SAMPLES]
        signal_power = float(np.mean(np.abs(reference.astype(np.complex128)) ** 2))
        noise_power = (
            signal_power
            / (10.0 ** (self._snr_db / 10.0))
            * (OUTPUT_SAMPLE_RATE / self._noise_bandwidth_hz)
        )
        self._noise_sigma = math.sqrt(noise_power / 2.0)
        return True

    def _add_noise(self, samples: np.ndarray) -> np.ndarray:
        if self._noise_sigma is None:
            return samples
        noise = self._noise_rng.standard_normal(
            (samples.size, 2), dtype=np.float32
        ) * np.float32(self._noise_sigma)
        return samples + (noise[:, 0] + 1j * noise[:, 1]).astype(np.complex64)

    def _emit_available(self) -> None:
        if not self._noise_ready():
            return
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
                self._sample_clock_profile.values(time_seconds)
                if self._sample_clock_profile is not None
                else self._sample_clock_ppm
                + self._sample_clock_drift_ppm_per_minute
                * time_seconds
                / 60.0
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

            if self._lo_ppm_profile is not None:
                phase = (
                    2.0
                    * np.pi
                    * self._center_frequency_hz
                    * self._lo_ppm_profile.integral(time_seconds)
                    * 1.0e-6
                )
                impaired *= np.exp(1j * phase).astype(np.complex64)
            elif self._lo_offset_hz != 0.0 or self._lo_drift_hz_per_minute != 0.0:
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

            self._write(self._add_noise(impaired))
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
    sample_clock_profile: PpmProfile | None = None,
) -> int:
    """Return enough ideal samples to cover the receiver-clock time warp."""

    duration_seconds = output_samples / OUTPUT_SAMPLE_RATE
    final_ppm = (
        sample_clock_ppm + sample_clock_drift_ppm_per_minute * duration_seconds / 60.0
    )
    minimum_ppm = (
        sample_clock_profile.minimum
        if sample_clock_profile is not None
        else min(sample_clock_ppm, final_ppm)
    )
    minimum_clock_scale = 1.0 + minimum_ppm * 1.0e-6
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
        "--sample-clock-knot",
        action="append",
        metavar="SECONDS:PPM",
        help=(
            "piecewise-linear sample-clock profile; repeat for each knot, "
            "starting at 0 seconds"
        ),
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
    parser.add_argument(
        "--lo-ppm-knot",
        action="append",
        metavar="SECONDS:PPM",
        help=(
            "piecewise-linear LO error relative to --center-frequency; "
            "repeat for each knot, starting at 0 seconds"
        ),
    )
    parser.add_argument(
        "--channel-profile",
        metavar="ricean|rayleigh|FILE[:NAME]",
        help=(
            "multipath channel: the EN 300 744 Annex B fixed Ricean (F1) or "
            "Rayleigh (P1) profile, or a measured profile written by "
            "tools/summarize_channel_profile.py (NAME selects one entry of "
            "the file, default the first)"
        ),
    )
    parser.add_argument(
        "--channel-tap",
        action="append",
        metavar="DELAY_US:POWER_DB[:PHASE_DEG[:DOPPLER_HZ]]",
        help=(
            "add one echo to a 0 dB direct path; repeat per echo "
            "(for example 0 dB echo tests: --channel-tap 50:0)"
        ),
    )
    parser.add_argument(
        "--channel-min-presence",
        type=float,
        default=0.5,
        help="skip measured-profile taps seen in fewer frames than this fraction (default: 0.5)",
    )
    parser.add_argument(
        "--channel-phase-seed",
        type=int,
        default=1,
        help="seed of the pseudo-random phases given to measured-profile taps",
    )
    parser.add_argument(
        "--snr-db",
        type=float,
        help=(
            "add white Gaussian noise so that C/N in the occupied signal "
            "bandwidth is this many dB (EN 300 744 convention)"
        ),
    )
    parser.add_argument(
        "--noise-seed", type=int, default=1, help="seed of the noise generator"
    )
    args = parser.parse_args()
    if args.duration <= 0.0:
        parser.error("duration must be positive")
    if args.channel_profile is not None and args.channel_tap:
        parser.error("--channel-profile cannot be combined with --channel-tap")
    args.channel_name = None
    args.channel_taps = None
    try:
        if args.channel_profile is not None:
            args.channel_name, args.channel_taps = load_channel_profile(
                args.channel_profile, args.channel_min_presence, args.channel_phase_seed
            )
        elif args.channel_tap:
            args.channel_name = "taps"
            args.channel_taps = normalize_taps(
                [ChannelTap(0.0, complex(1.0, 0.0), 0.0)]
                + [parse_channel_tap(text) for text in args.channel_tap]
            )
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exception:
        parser.error(f"invalid channel profile: {exception}")
    if args.snr_db is not None and not math.isfinite(args.snr_db):
        parser.error("--snr-db must be finite")
    impairment_values = (
        args.sample_clock_ppm,
        args.sample_clock_drift_ppm_per_minute,
        args.lo_offset_hz,
        args.lo_drift_hz_per_minute,
    )
    if not all(math.isfinite(value) for value in impairment_values):
        parser.error("clock impairment values must be finite")
    args.sample_clock_profile = parse_ppm_knots(
        parser,
        args.sample_clock_knot,
        "--sample-clock-knot",
        args.duration,
    )
    args.lo_ppm_profile = parse_ppm_knots(
        parser, args.lo_ppm_knot, "--lo-ppm-knot", args.duration
    )
    if args.sample_clock_profile is not None and (
        args.sample_clock_ppm != 0.0
        or args.sample_clock_drift_ppm_per_minute != 0.0
    ):
        parser.error(
            "sample-clock knots cannot be combined with the legacy "
            "constant/linear sample-clock options"
        )
    if args.lo_ppm_profile is not None and (
        args.lo_offset_hz != 0.0 or args.lo_drift_hz_per_minute != 0.0
    ):
        parser.error(
            "LO ppm knots cannot be combined with the legacy constant/linear "
            "LO options"
        )
    duration_seconds = args.duration
    final_clock_ppm = args.sample_clock_ppm + (
        args.sample_clock_drift_ppm_per_minute * duration_seconds / 60.0
    )
    minimum_clock_ppm = (
        args.sample_clock_profile.minimum
        if args.sample_clock_profile is not None
        else min(args.sample_clock_ppm, final_clock_ppm)
    )
    if 1.0 + minimum_clock_ppm * 1.0e-6 <= 0.0:
        parser.error("sample-clock impairment makes the clock non-positive")
    return args


def occupied_bandwidth_hz(config: DvbtConfig) -> float:
    """Span of the active carriers: (K - 1) / Tu."""
    _, fft_length, _ = TRANSMISSION_MODES[config.transmission_mode]
    elementary_rate = config.bandwidth_hz * 8.0 / 7.0
    return (ACTIVE_CARRIERS[config.transmission_mode] - 1) * elementary_rate / fft_length


def channel_metadata(args: argparse.Namespace, config: DvbtConfig) -> dict:
    return {
        "profile": args.channel_name,
        "taps": (
            [
                {
                    "delay_us": tap.delay_us,
                    "power_db": tap.power_db,
                    "phase_deg": tap.phase_deg,
                    "doppler_hz": tap.doppler_hz,
                }
                for tap in args.channel_taps
            ]
            if args.channel_taps is not None
            else None
        ),
        "snr_db": args.snr_db,
        "snr_reference_bandwidth_hz": occupied_bandwidth_hz(config),
        "noise_seed": args.noise_seed if args.snr_db is not None else None,
    }


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
        args.sample_clock_profile,
    )
    channel = (
        MultipathChannel(args.channel_taps, OUTPUT_SAMPLE_RATE)
        if args.channel_taps is not None
        else None
    )
    sink = StreamingCs16Sink(
        output,
        sample_count,
        args.sample_clock_ppm,
        args.sample_clock_drift_ppm_per_minute,
        args.lo_offset_hz,
        args.lo_drift_hz_per_minute,
        args.sample_clock_profile,
        args.lo_ppm_profile,
        args.center_frequency,
        channel,
        args.snr_db,
        occupied_bandwidth_hz(config),
        args.noise_seed,
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
            "sample_clock_knots_ppm": (
                [list(knot) for knot in args.sample_clock_profile.knots]
                if args.sample_clock_profile is not None
                else None
            ),
            "lo_knots_ppm": (
                [list(knot) for knot in args.lo_ppm_profile.knots]
                if args.lo_ppm_profile is not None
                else None
            ),
        },
        "channel": channel_metadata(args, config),
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
    sample_clock_description = (
        f"knots={args.sample_clock_profile.knots}"
        if args.sample_clock_profile is not None
        else f"{args.sample_clock_ppm:+g} ppm, "
        f"drift={args.sample_clock_drift_ppm_per_minute:+g} ppm/min"
    )
    lo_description = (
        f"knots={args.lo_ppm_profile.knots} ppm at {args.center_frequency} Hz"
        if args.lo_ppm_profile is not None
        else f"{args.lo_offset_hz:+g} Hz, "
        f"drift={args.lo_drift_hz_per_minute:+g} Hz/min"
    )
    print(
        f"impairments: sample clock {sample_clock_description}; "
        f"LO {lo_description}",
        file=sys.stderr,
    )
    if args.channel_taps is not None or args.snr_db is not None:
        channel_description = (
            f"{args.channel_name} ({len(args.channel_taps)} taps)"
            if args.channel_taps is not None
            else "ideal"
        )
        snr_description = (
            f"{args.snr_db:g} dB in {occupied_bandwidth_hz(config) / 1e6:.3f} MHz"
            if args.snr_db is not None
            else "none"
        )
        print(
            f"channel: {channel_description}; AWGN C/N {snr_description}",
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
