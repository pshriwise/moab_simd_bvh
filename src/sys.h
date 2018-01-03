
#pragma once

#include <xmmintrin.h>

#define __forceinline inline __attribute__((always_inline))

#define __aligned(...) __attribute__((aligned(__VA_ARGS__)))

inline void prefetchL1 (const void* ptr) { _mm_prefetch((const char*)ptr, _MM_HINT_T0); }
inline void prefetchL2 (const void* ptr) { _mm_prefetch((const char*)ptr, _MM_HINT_T1); }
