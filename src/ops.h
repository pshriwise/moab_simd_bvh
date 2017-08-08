
#pragma once

#include <immintrin.h>
#include <xmmintrin.h>

#pragma intrinsic(_BitScanForward)

inline size_t __bsf(size_t v) {
  unsigned long r = 0; r = __builtin_ffs(v)-1; return r;
}

inline size_t __bscf(size_t& v) {
  size_t i = __bsf(v);
  v &= v-1;
  return i;
}

