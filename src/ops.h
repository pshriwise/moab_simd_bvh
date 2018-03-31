
#pragma once

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

__forceinline float rcp ( const float x ) {
  float val = x < min_rcp_input ? min_rcp_input : x;
  return 1.0f / val;
}

/* __forceinline float rcp ( const float x ) { */
/*   const __m128 a = _mm_set_ss(x); */

/*   const __m128 r = _mm_rcp_ps(a); */

/*   return _mm_cvtss_f32(_mm_mul_ps(r,_mm_sub_ps(_mm_set_ss(2.0f), _mm_mul_ps(r, a)))); */
/* } */
