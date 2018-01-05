#pragma once


#include "constants.h"
#include "Vec3.h"
#include "math.h"

struct vbool4
{

  enum { size = 4 }; // number of elements
  union { __m128i v; bool b[4]; }; // data holder

  inline vbool4 () {}

  inline vbool4 (const __m128i &a) : v(a) {}

  inline vbool4 (const vbool4& other) { b[0] = other.b[0];
                                        b[1] = other.b[1];
					b[2] = other.b[2];
					b[3] = other.b[2]; }

  inline vbool4& operator =( const bool& a ) { b[0] = a; b[1] = a; b[2] = a; b[3] = a; }
    
  inline vbool4( bool a )                        { b[0] = a; b[1] = a; b[2] = a; b[3] = a; }
  inline vbool4( bool i, bool j, bool k, bool l) { b[0] = i; b[1] = j; b[2] = k; b[3] = l; }

  static inline vbool4 load ( const void* const a) { vbool4 v;
                                              for(size_t i=0; i < 4; i++) {
						v.b[i] = *(bool*)((const char*)a+i*sizeof(bool));
					      }
                                              return v;}
  inline const bool& operator [](const size_t index) const { assert(index < 4); return b[index]; }
  inline       bool& operator [](const size_t index)       { assert(index < 4); return b[index]; }

  

};

template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

inline const size_t movemask( const vbool4& v ) { return sgn(v.b[3]) << 3 | sgn(v.b[2]) << 2 | sgn(v.b[1]) << 1 | sgn(v.b[0]); }

inline bool all( const vbool4& v ) { return v.b[0] && v.b[1] && v.b[2] && v.b[3]; }

/* ////////// Unary Ops ////////// */
/* inline const vbool4 operator +( const vbool4& a ) { return a; } */
/* inline const vbool4 operator -( const vbool4& a ) { return vbool4(-a[0],-a[1],-a[2],-a[3]); } */

/* ////////// Binary Ops ////////// */
/* inline const vbool4 operator +( const vbool4& a, const vbool4& b ) { return vbool4(a[0]+b[0],a[1]+b[1],a[2]+b[2],a[3]+b[3]); } */
/* inline const vbool4 operator +( const vbool4& a, const bool& b ) { return a+ vbool4(b); } */
/* inline const vbool4 operator +( const bool& a, const vbool4& b ) { return vbool4(a)+b; } */

/* inline const vbool4 operator -( const vbool4& a, const vbool4& b ) { return vbool4(a[0]-b[0],a[1]-b[1],a[2]-b[2],a[3]-b[3]); } */
/* inline const vbool4 operator -( const vbool4& a, const bool& b ) { return a-vbool4(b); } */
/* inline const vbool4 operator -( const bool& a, const vbool4& b ) { return vbool4(a)-b; } */

/* inline const vbool4 operator *( const vbool4& a, const vbool4& b ) { return vbool4(a[0]*b[0],a[1]*b[1],a[2]*b[2],a[3]*b[3]); } */
/* inline const vbool4 operator *( const vbool4& a, const bool& b ) { return a*vbool4(b); } */
/* inline const vbool4 operator *( const bool& a, const vbool4& b ) { return vbool4(a)*b; } */

/* inline const vbool4 operator /( const vbool4& a, const vbool4& b ) { return vbool4(a[0]/b[0],a[1]/b[1],a[2]/b[2],a[3]/b[3]); } */
/* inline const vbool4 operator /( const vbool4& a, const bool& b ) { return a/vbool4(b); } */
/* inline const vbool4 operator /( const bool& a, const vbool4& b ) { return vbool4(a)/b; } */

/* ////////// Assignment Ops ////////// */
/* inline vbool4& operator +=( vbool4& a, const vbool4& b ) { return a = a + b; } */
/* inline vbool4& operator +=( vbool4& a, const bool&   b ) { return a = a + b; } */

/* inline vbool4& operator -=( vbool4& a, const vbool4& b ) { return a = a - b; } */
/* inline vbool4& operator -=( vbool4& a, const bool&   b ) { return a = a - b; } */

/* inline vbool4& operator *=( vbool4& a, const vbool4& b ) { return a = a * b; } */
/* inline vbool4& operator *=( vbool4& a, const bool&   b ) { return a = a * b; } */

/* inline vbool4& operator /=( vbool4& a, const vbool4& b ) { return a = a / b; } */
/* inline vbool4& operator /=( vbool4& a, const bool&   b ) { return a = a / b; }                        */

/* ////////// Other Common Ops ////////// */
/* inline const vbool4 madd  ( const vbool4& a, const vbool4& b, const vbool4& c) { return a*b+c; } */
/* inline const vbool4 msub  ( const vbool4& a, const vbool4& b, const vbool4& c) { return a*b-c; } */
/* inline const vbool4 nmadd ( const vbool4& a, const vbool4& b, const vbool4& c) { return -a*b+c;} */
/* inline const vbool4 nmsub ( const vbool4& a, const vbool4& b, const vbool4& c) { return -a*b-c; }        */

/* inline bool min(const vbool4& v ) { return std::min(std::min(v.b[0],v.b[1]),std::min(v.b[2],v.b[3])); } */
/* inline bool max(const vbool4& v ) { return std::max(std::max(v.b[0],v.b[1]),std::max(v.b[2],v.b[3])); } */


inline std::ostream& operator<<(std::ostream& cout, const vbool4& a) {
   return cout << "<" << a[0] << ", " << a[1] << ", " << a[2] << ", " << a[3] << ">";
}                       
