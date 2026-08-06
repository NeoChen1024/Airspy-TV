# libfec Reed-Solomon subset

This directory contains the generic 8-bit Reed-Solomon encoder and decoder
from [ka9q/libfec](https://github.com/ka9q/libfec), reduced to the source files
needed by Airspy-TV.

Upstream commit: `7c6706fb969c3f8fe6ec7778b2472762e0d88acc`

The following files are copied from upstream, with trailing whitespace
normalized so they pass Airspy-TV's source checks:

- `init_rs_char.c`, `encode_rs_char.c`, and `decode_rs_char.c`
- `char.h` and `rs-common.h`
- `init_rs.h`, `encode_rs.h`, and `decode_rs.h`
- `LICENSE` (upstream `lesser.txt`)

`include/libfec_rs.h` and `CMakeLists.txt` are Airspy-TV integration files.
The implementation is licensed under the GNU Lesser General Public License,
version 2.1. See `LICENSE` for the complete terms.
