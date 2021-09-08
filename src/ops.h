#ifndef _OPS_H
#define _OPS_H

#include <immintrin.h>
#include <xmmintrin.h>

#pragma intrinsic(_BitScanForward)

__forceinline size_t __bsf(size_t v) {
#if defined(__AVX2__)
  return _tzcnt_u32(v);
#else
  unsigned long r = 0; r = __builtin_ffs(v)-1; return r;
#endif
}

__forceinline size_t __bscf(size_t& v) {
  size_t i = __bsf(v);
  v &= v-1;
  return i;
}

#endif
