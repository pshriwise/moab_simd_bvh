#ifndef _VINT_H
#define _VINT_H


#include "constants.h"
#include "Vec3.h"
#include "math.h"
#include "vbool.h"
#include "sys.h"
#include <immintrin.h>
#include <emmintrin.h>

struct vint4
{

  enum { size = 4 }; // number of elements
  union{ __m128i v; int f[4]; }; // data holder

  __forceinline vint4 () {}

  __forceinline vint4( const __m128i a ) : v(a) {}

  __forceinline operator const __m128i&( void ) const { return v; }
  __forceinline operator       __m128i&( void )       { return v; }

  __forceinline vint4 (const vint4& other) { f[0] = other.f[0];
                                          f[1] = other.f[1];
					  f[2] = other.f[2];
					  f[3] = other.f[3]; }

  __forceinline vint4& operator =( const int& a ) { f[0] = a; f[1] = a; f[2] = a; f[3] = a; }

  __forceinline vint4( int a )                            { f[0] = a; f[1] = a; f[2] = a; f[3] = a; }
  __forceinline vint4( int a, int b, int c, int d) {  f[0] = a; f[1] = b; f[2] = c; f[3] = d; }

  __forceinline const int& operator [](const size_t index) const { assert(index < 4); return f[index]; }
  __forceinline       int& operator [](const size_t index)       { assert(index < 4); return f[index]; }

};


////////// Comparators //////////
__forceinline const vbool4 operator ==( const vint4& a, const vint4&b ) { return vbool4(a.f[0]==b.f[0],
										     a.f[1]==b.f[1],
										     a.f[2]==b.f[2],
										     a.f[3]==b.f[3]); }
__forceinline const vbool4 operator !=( const vint4& a, const vint4&b ) { return vbool4(a.f[0]!=b.f[0],
										     a.f[1]!=b.f[1],
										     a.f[2]!=b.f[2],
										     a.f[3]!=b.f[3]); }
__forceinline const vbool4 operator <( const vint4& a, const vint4&b ) { return vbool4(a.f[0]<b.f[0],
										     a.f[1]<b.f[1],
										     a.f[2]<b.f[2],
										     a.f[3]<b.f[3]); }
__forceinline const vbool4 operator >=( const vint4& a, const vint4&b ) { return vbool4(a.f[0]>=b.f[0],
										     a.f[1]>=b.f[1],
										     a.f[2]>=b.f[2],
										     a.f[3]>=b.f[3]); }
__forceinline const vbool4 operator >( const vint4& a, const vint4&b ) { return vbool4(a.f[0]>b.f[0],
										     a.f[1]>b.f[1],
										     a.f[2]>b.f[2],
										     a.f[3]>b.f[3]); }
__forceinline const vbool4 operator <=( const vint4& a, const vint4&b ) { return vbool4(a.f[0]<=b.f[0],
										     a.f[1]<=b.f[1],
										     a.f[2]<=b.f[2],
										     a.f[3]<=b.f[3]); }


__forceinline int min(const vint4& v ) { return std::min(std::min(v.f[0],v.f[1]),std::min(v.f[2],v.f[3])); }
__forceinline int max(const vint4& v ) { return std::max(std::max(v.f[0],v.f[1]),std::max(v.f[2],v.f[3])); }

__forceinline vint4 min(const vint4& a, const vint4&b) { return vint4(std::min(a.f[0],b.f[0]),
				                                       std::min(a.f[1],b.f[1]),
								       std::min(a.f[2],b.f[2]),
								       std::min(a.f[3],b.f[3])); }

__forceinline vint4 min(const vint4& a, const vint4&b, const vint4& c) { return min(min(a,b),c); }

__forceinline vint4 min(const vint4& a, const vint4&b, const vint4& c, const vint4& d) { return min(min(a,b),c,d); }


__forceinline vint4 max(const vint4& a, const vint4&b) { vint4 v;
                                                        v.f[0] = std::max(a.f[0],b.f[0]);
                                                        v.f[1] = std::max(a.f[1],b.f[1]);
                                                        v.f[2] = std::max(a.f[2],b.f[2]);
                                                        v.f[3] = std::max(a.f[3],b.f[3]);
							return v;
                                                       }

__forceinline std::ostream& operator<<(std::ostream& cout, const vint4& a) {
   return cout << "<" << a[0] << ", " << a[1] << ", " << a[2] << ", " << a[3] << ">";
}

#endif
