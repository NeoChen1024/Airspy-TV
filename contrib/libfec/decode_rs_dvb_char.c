/* DVB-T/C shortened RS(204,188) specialization of Phil Karn's generic
 * decoder. The algorithm remains decode_rs.h; fixing the code parameters here
 * lets the compiler fold loop bounds and field-exponent arithmetic that are
 * runtime values in decode_rs_char().
 */

#include <stdint.h>
#include <string.h>
#include <time.h>

#include "libfec_rs.h"

typedef unsigned char data_t;
#include "rs-common.h"

#define NN 255
#define NROOTS 16
#define PAD 51
#define FCR 0
#define PRIM 1
#define IPRIM 1
#define ALPHA_TO (rs->alpha_to)
#define INDEX_OF (rs->index_of)

static inline int dvb_modnn(int value) {
  while (value >= NN) {
    value -= NN;
    value = (value >> 8) + (value & NN);
  }
  return value;
}

#define MODNN(value) dvb_modnn(value)

static int is_dvb_codec(const struct rs *rs) {
  return rs != NULL && rs->nn == NN && rs->nroots == NROOTS &&
         rs->pad == PAD && rs->fcr == FCR && rs->prim == PRIM;
}

int decode_rs_dvb_char(void *p, data_t *data) {
  int retval;
  struct rs *rs = (struct rs *)p;
  int *eras_pos = NULL;
  int no_eras = 0;
  if (!is_dvb_codec(rs) || data == NULL)
    return -1;

#include "decode_rs.h"

  return retval;
}

static uint64_t rs_profile_now_ns(void) {
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return ((uint64_t)now.tv_sec * UINT64_C(1000000000)) + (uint64_t)now.tv_nsec;
}

int decode_rs_dvb_char_profiled(void *p, data_t *data,
                                struct libfec_rs_decode_profile *profile) {
  int retval;
  struct rs *rs = (struct rs *)p;
  int *eras_pos = NULL;
  int no_eras = 0;
  if (!is_dvb_codec(rs) || data == NULL || profile == NULL)
    return -1;
  uint64_t profile_phase_started = rs_profile_now_ns();

  memset(profile, 0, sizeof(*profile));
#define RS_DECODE_SYNDROME_DONE(syn_error_value) do {                         \
    const uint64_t profile_now = rs_profile_now_ns();                         \
    profile->syndrome_ns = profile_now - profile_phase_started;               \
    profile->clean_codeword = !(syn_error_value);                             \
    profile_phase_started = profile_now;                                      \
  } while (0)
#define RS_DECODE_LOCATOR_DONE(result_value) do {                             \
    (void)(result_value);                                                      \
    const uint64_t profile_now = rs_profile_now_ns();                         \
    profile->error_locator_ns = profile_now - profile_phase_started;          \
    profile_phase_started = profile_now;                                      \
  } while (0)
#define RS_DECODE_CORRECTION_DONE(result_value) do {                          \
    (void)(result_value);                                                      \
    profile->correction_ns = rs_profile_now_ns() - profile_phase_started;     \
  } while (0)

#include "decode_rs.h"

  return retval;
}

#undef MODNN
#undef INDEX_OF
#undef ALPHA_TO
#undef IPRIM
#undef PRIM
#undef FCR
#undef PAD
#undef NROOTS
#undef NN
