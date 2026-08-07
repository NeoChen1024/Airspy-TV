# solid-resampler

The name is an engineering joke about the implementation's liquid-dsp origin.
This directory contains a narrow frequency-translating arbitrary-rate
resampler implementation derived from the filter-design and complex/real
dot-product techniques in [liquid-dsp](https://github.com/jgaeddert/liquid-dsp).

Upstream commit: `6bdbc8e79429c07275cc85a28688fb787ff44133`

The implementation retains the relevant MIT notices and design provenance but
is intentionally not API- or source-compatible with liquid-dsp. Airspy-TV owns
the modified implementation and does not track upstream changes.

The retained concepts are:

- Kaiser-windowed sinc prototype design and polyphase coefficient layout;
- complex-input, real-coefficient portable and SIMD dot products;
- fixed-point phase accumulation for arbitrary-rate resampling.

Airspy-TV replaces liquid-dsp's input-driven mutable filter bank with a Q32.32
output-driven scheduler. Output ranges are independent and run on a persistent
worker pool. An optional output mixer provides phase-continuous frequency
translation in the same output pass. Only this common resampler is included;
liquid-dsp's buffers, logging, generic filters, FFT, modem, and other DSP
modules are not vendored.

Callers specify absolute input/output rates, passband and stopband edges, and a
target attenuation. The implementation derives the Kaiser filter length from
the transition width and rounds it up to a 16-tap boundary for SIMD kernels.
The default attenuation target is 80 dB; consumers remain responsible for
supplying spectral edges appropriate to their standard and sample rates.

The code is licensed under the MIT License. See `LICENSE`.
