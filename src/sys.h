



#include <xmmintrin.h>



inline void prefetchL1 (const void* ptr) { _mm_prefetch((const char*)ptr, _MM_HINT_T0); }
inline void prefetchL2 (const void* ptr) { _mm_prefetch((const char*)ptr, _MM_HINT_T1); }
