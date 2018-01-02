#pragma once


#include "constants.h"
#include "Vec3.h"
#include "math.h"
#include "vbool.h"
#include <xmmintrin.h>

struct vfloat4
{

  enum { size = 4 }; // number of elements
  union { __m128 v; float f[4]; int i[4]; }; // data holder

  //  inline vfloat4 (float a) : v(_mm_set1_ps(a) {}
  inline vfloat4 (const __m128 a) : v(a) {}

  inline operator const __m128&( void ) const { return v; }
  inline operator       __m128&( void )       { return v; }
  
  inline vfloat4 () {}
  inline vfloat4 (const vfloat4& other) { f[0] = other.f[0];
                                          f[1] = other.f[1];
					  f[2] = other.f[2];
					  f[3] = other.f[3]; }

  inline vfloat4& operator =( const float& a ) { f[0] = a; f[1] = a; f[2] = a; f[3] = a; }
    
  inline vfloat4( float a )                            { f[0] = a; f[1] = a; f[2] = a; f[3] = a; }
  inline vfloat4( float a, float b, float c, float d) {  f[0] = a; f[1] = b; f[2] = c; f[3] = d; }

  inline const float& operator [](const size_t index) const { assert(index < 4); return f[index]; }
  inline       float& operator [](const size_t index)       { assert(index < 4); return f[index]; }

  static inline vfloat4 load ( const void* const a) { return _mm_load_ps((float*)a); }


  static inline void store ( void* ptr, const vfloat4&v ) { return _mm_store_ps((float*)ptr, v); }

};


////////// Unary Ops //////////
inline const vfloat4 operator +( const vfloat4& a ) { return a; }
inline const vfloat4 operator -( const vfloat4& a ) { return vfloat4(-a[0],-a[1],-a[2],-a[3]); }

////////// Binary Ops //////////
inline const vfloat4 operator +( const vfloat4& a, const vfloat4& b ) { return _mm_add_ps(a.v, b.v); }
inline const vfloat4 operator +( const vfloat4& a, const float& b ) { return a + vfloat4(b); }
inline const vfloat4 operator +( const float& a, const vfloat4& b ) { return vfloat4(a) + b; }

inline const vfloat4 operator -( const vfloat4& a, const vfloat4& b ) { return _mm_sub_ps(a.v, b.v); }
inline const vfloat4 operator -( const vfloat4& a, const float& b ) { return a - vfloat4(b); }
inline const vfloat4 operator -( const float& a, const vfloat4& b ) { return vfloat4(a)-b; }

inline const vfloat4 operator *( const vfloat4& a, const vfloat4& b ) { return _mm_mul_ps(a.v, b.v); }
inline const vfloat4 operator *( const vfloat4& a, const float& b ) { return a*vfloat4(b); }
inline const vfloat4 operator *( const float& a, const vfloat4& b ) { return vfloat4(a)*b; }

inline const vfloat4 operator /( const vfloat4& a, const vfloat4& b ) { return _mm_div_ps(a.v, b.v); }
inline const vfloat4 operator /( const vfloat4& a, const float& b ) { return a/vfloat4(b); }
inline const vfloat4 operator /( const float& a, const vfloat4& b ) { return vfloat4(a)/b; }

////////// Assignment Ops //////////
inline vfloat4& operator +=( vfloat4& a, const vfloat4& b ) { return a = a + b; }
inline vfloat4& operator +=( vfloat4& a, const float&   b ) { return a = a + b; }

inline vfloat4& operator -=( vfloat4& a, const vfloat4& b ) { return a = a - b; }
inline vfloat4& operator -=( vfloat4& a, const float&   b ) { return a = a - b; }

inline vfloat4& operator *=( vfloat4& a, const vfloat4& b ) { return a = a * b; }
inline vfloat4& operator *=( vfloat4& a, const float&   b ) { return a = a * b; }

inline vfloat4& operator /=( vfloat4& a, const vfloat4& b ) { return a = a / b; }
inline vfloat4& operator /=( vfloat4& a, const float&   b ) { return a = a / b; }                       


////////// Comparators //////////
inline const vbool4 operator ==( const vfloat4& a, const vfloat4&b ) { return vbool4(a.f[0]==b.f[0],
										     a.f[1]==b.f[1],
										     a.f[2]==b.f[2],
										     a.f[3]==b.f[3]); }
inline const vbool4 operator !=( const vfloat4& a, const vfloat4&b ) { return vbool4(a.f[0]!=b.f[0],
										     a.f[1]!=b.f[1],
										     a.f[2]!=b.f[2],
										     a.f[3]!=b.f[3]); }
inline const vbool4 operator <( const vfloat4& a, const vfloat4&b ) { return vbool4(a.f[0]<b.f[0],
										     a.f[1]<b.f[1],
										     a.f[2]<b.f[2],
										     a.f[3]<b.f[3]); }
inline const vbool4 operator >=( const vfloat4& a, const vfloat4&b ) { return vbool4(a.f[0]>=b.f[0],
										     a.f[1]>=b.f[1],
										     a.f[2]>=b.f[2],
										     a.f[3]>=b.f[3]); }
inline const vbool4 operator >( const vfloat4& a, const vfloat4&b ) { return vbool4(a.f[0]>b.f[0],
										     a.f[1]>b.f[1],
										     a.f[2]>b.f[2],
										     a.f[3]>b.f[3]); }
inline const vbool4 operator <=( const vfloat4& a, const vfloat4&b ) { return vbool4(a.f[0]<=b.f[0],
										     a.f[1]<=b.f[1],
										     a.f[2]<=b.f[2],
										     a.f[3]<=b.f[3]); }

////////// Other Common Ops //////////
inline const vfloat4 madd  ( const vfloat4& a, const vfloat4& b, const vfloat4& c) { return a*b+c; }
inline const vfloat4 msub  ( const vfloat4& a, const vfloat4& b, const vfloat4& c) { return a*b-c; }
inline const vfloat4 nmadd ( const vfloat4& a, const vfloat4& b, const vfloat4& c) { return -a*b+c;}
inline const vfloat4 nmsub ( const vfloat4& a, const vfloat4& b, const vfloat4& c) { return -a*b-c; }       

inline float min(const vfloat4& v ) { return std::min(std::min(v.f[0],v.f[1]),std::min(v.f[2],v.f[3])); }
inline float max(const vfloat4& v ) { return std::max(std::max(v.f[0],v.f[1]),std::max(v.f[2],v.f[3])); }

inline vfloat4 min(const vfloat4& a, const vfloat4&b) { return _mm_min_ps(a.v, b.v); }
  /* return vfloat4(std::min(a.f[0],b.f[0]), */
  /* 		 std::min(a.f[1],b.f[1]), */
  /* 		 std::min(a.f[2],b.f[2]), */
  /* 		 std::min(a.f[3],b.f[3])); } */

inline vfloat4 min(const vfloat4& a, const vfloat4&b, const vfloat4& c) { return min(min(a,b),c); }

inline vfloat4 min(const vfloat4& a, const vfloat4&b, const vfloat4& c, const vfloat4& d) { return min(min(a,b),c,d); }


inline vfloat4 max(const vfloat4& a, const vfloat4&b) { return _mm_max_ps(a.v, b.v); }
/*   vfloat4 v; */
/*   v.f[0] = std::max(a.f[0],b.f[0]); */
/*   v.f[1] = std::max(a.f[1],b.f[1]); */
/*   v.f[2] = std::max(a.f[2],b.f[2]); */
/*   v.f[3] = std::max(a.f[3],b.f[3]); */
/*   return v; */
/* } */

inline vfloat4 max(const vfloat4& a, const vfloat4&b, const vfloat4& c) { return max(max(a,b),c); }

inline vfloat4 max(const vfloat4& a, const vfloat4&b, const vfloat4& c, const vfloat4& d) { return max(max(a,b),c,d); }

inline std::ostream& operator<<(std::ostream& cout, const vfloat4& a) {
   return cout << "<" << a[0] << ", " << a[1] << ", " << a[2] << ", " << a[3] << ">";
}                       
