/*
 * Minimal public interface for Phil Karn's generic 8-bit Reed-Solomon codec.
 *
 * The implementation is vendored from ka9q/libfec and may be used under the
 * terms of the GNU Lesser General Public License, version 2.1.
 */
#ifndef LIBFEC_RS_H
#define LIBFEC_RS_H

#ifdef __cplusplus
extern "C" {
#endif

void *init_rs_char(int symsize, int gfpoly, int fcr, int prim, int nroots,
                   int pad);
void free_rs_char(void *rs);
void encode_rs_char(void *rs, unsigned char *data, unsigned char *parity);
int decode_rs_char(void *rs, unsigned char *data, int *eras_pos, int no_eras);

#ifdef __cplusplus
}
#endif

#endif
