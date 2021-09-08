#ifndef _VDOUBLE_H
#define _VDOUBLE_H


#include "constants.h"
#include "Vec3.h"
#include "math.h"
#include "vbool.h"
#include "sys.h"

struct vdouble4
{

  enum { size = 4 }; // number of elements
  double f[4]; // data holder

  __forceinline vdouble4 () {}
  __forceinline vdouble4 (const vdouble4& other) { f[0] = other.f[0];
                                          f[1] = other.f[1];
					  f[2] = other.f[2];
					  f[3] = other.f[3]; }

  __forceinline vdouble4& operator =( const double& a ) { f[0] = a; f[1] = a; f[2] = a; f[3] = a; }

  __forceinline vdouble4( double a )                            { f[0] = a; f[1] = a; f[2] = a; f[3] = a; }
  __forceinline vdouble4( double a, double b, double c, double d) {  f[0] = a; f[1] = b; f[2] = c; f[3] = d; }

  __forceinline const double& operator [](const size_t index) const { assert(index < 4); return f[index]; }
  __forceinline       double& operator [](const size_t index)       { assert(index < 4); return f[index]; }

  static __forceinline vdouble4 load ( const void* const a) { vdouble4 v;
                                               for(size_t i=0; i < 4; i++) {
						 v.f[i] = *(double*)((const char*)a+i*sizeof(double));
					       }
					       return v; }

  static __forceinline void store ( void* ptr, const vdouble4&v ) {
                                               vdouble4* p = (vdouble4*) ptr;
                                               for(size_t i=0; i < 4; i++) {
						 (*p).f[i] = v[i];
					       }
                            }

};


////////// Unary Ops //////////
__forceinline const vdouble4 operator +( const vdouble4& a ) { return a; }
__forceinline const vdouble4 operator -( const vdouble4& a ) { return vdouble4(-a[0],-a[1],-a[2],-a[3]); }

////////// Binary Ops //////////
__forceinline const vdouble4 operator +( const vdouble4& a, const vdouble4& b ) { return vdouble4(a[0]+b[0],a[1]+b[1],a[2]+b[2],a[3]+b[3]); }
__forceinline const vdouble4 operator +( const vdouble4& a, const double& b ) { return a+ vdouble4(b); }
__forceinline const vdouble4 operator +( const double& a, const vdouble4& b ) { return vdouble4(a)+b; }

__forceinline const vdouble4 operator -( const vdouble4& a, const vdouble4& b ) { return vdouble4(a[0]-b[0],a[1]-b[1],a[2]-b[2],a[3]-b[3]); }
__forceinline const vdouble4 operator -( const vdouble4& a, const double& b ) { return a-vdouble4(b); }
__forceinline const vdouble4 operator -( const double& a, const vdouble4& b ) { return vdouble4(a)-b; }

__forceinline const vdouble4 operator *( const vdouble4& a, const vdouble4& b ) { return vdouble4(a[0]*b[0],a[1]*b[1],a[2]*b[2],a[3]*b[3]); }
__forceinline const vdouble4 operator *( const vdouble4& a, const double& b ) { return a*vdouble4(b); }
__forceinline const vdouble4 operator *( const double& a, const vdouble4& b ) { return vdouble4(a)*b; }

__forceinline const vdouble4 operator /( const vdouble4& a, const vdouble4& b ) { return vdouble4(a[0]/b[0],a[1]/b[1],a[2]/b[2],a[3]/b[3]); }
__forceinline const vdouble4 operator /( const vdouble4& a, const double& b ) { return a/vdouble4(b); }
__forceinline const vdouble4 operator /( const double& a, const vdouble4& b ) { return vdouble4(a)/b; }

////////// Assignment Ops //////////
__forceinline vdouble4& operator +=( vdouble4& a, const vdouble4& b ) { return a = a + b; }
__forceinline vdouble4& operator +=( vdouble4& a, const double&   b ) { return a = a + b; }

__forceinline vdouble4& operator -=( vdouble4& a, const vdouble4& b ) { return a = a - b; }
__forceinline vdouble4& operator -=( vdouble4& a, const double&   b ) { return a = a - b; }

__forceinline vdouble4& operator *=( vdouble4& a, const vdouble4& b ) { return a = a * b; }
__forceinline vdouble4& operator *=( vdouble4& a, const double&   b ) { return a = a * b; }

__forceinline vdouble4& operator /=( vdouble4& a, const vdouble4& b ) { return a = a / b; }
__forceinline vdouble4& operator /=( vdouble4& a, const double&   b ) { return a = a / b; }


////////// Comparators //////////
__forceinline const vbool4 operator ==( const vdouble4& a, const vdouble4&b ) { return vbool4(a.f[0]==b.f[0],
										     a.f[1]==b.f[1],
										     a.f[2]==b.f[2],
										     a.f[3]==b.f[3]); }
__forceinline const vbool4 operator !=( const vdouble4& a, const vdouble4&b ) { return vbool4(a.f[0]!=b.f[0],
										     a.f[1]!=b.f[1],
										     a.f[2]!=b.f[2],
										     a.f[3]!=b.f[3]); }
__forceinline const vbool4 operator <( const vdouble4& a, const vdouble4&b ) { return vbool4(a.f[0]<b.f[0],
										     a.f[1]<b.f[1],
										     a.f[2]<b.f[2],
										     a.f[3]<b.f[3]); }
__forceinline const vbool4 operator >=( const vdouble4& a, const vdouble4&b ) { return vbool4(a.f[0]>=b.f[0],
										     a.f[1]>=b.f[1],
										     a.f[2]>=b.f[2],
										     a.f[3]>=b.f[3]); }
__forceinline const vbool4 operator >( const vdouble4& a, const vdouble4&b ) { return vbool4(a.f[0]>b.f[0],
										     a.f[1]>b.f[1],
										     a.f[2]>b.f[2],
										     a.f[3]>b.f[3]); }
__forceinline const vbool4 operator <=( const vdouble4& a, const vdouble4&b ) { return vbool4(a.f[0]<=b.f[0],
										     a.f[1]<=b.f[1],
										     a.f[2]<=b.f[2],
										     a.f[3]<=b.f[3]); }

////////// Other Common Ops //////////
__forceinline const vdouble4 madd  ( const vdouble4& a, const vdouble4& b, const vdouble4& c) { return a*b+c; }
__forceinline const vdouble4 msub  ( const vdouble4& a, const vdouble4& b, const vdouble4& c) { return a*b-c; }
__forceinline const vdouble4 nmadd ( const vdouble4& a, const vdouble4& b, const vdouble4& c) { return -a*b+c;}
__forceinline const vdouble4 nmsub ( const vdouble4& a, const vdouble4& b, const vdouble4& c) { return -a*b-c; }

__forceinline double min(const vdouble4& v ) { return std::min(std::min(v.f[0],v.f[1]),std::min(v.f[2],v.f[3])); }
__forceinline double max(const vdouble4& v ) { return std::max(std::max(v.f[0],v.f[1]),std::max(v.f[2],v.f[3])); }

__forceinline vdouble4 min(const vdouble4& a, const vdouble4&b) { return vdouble4(std::min(a.f[0],b.f[0]),
				                                       std::min(a.f[1],b.f[1]),
								       std::min(a.f[2],b.f[2]),
								       std::min(a.f[3],b.f[3])); }

__forceinline vdouble4 min(const vdouble4& a, const vdouble4&b, const vdouble4& c) { return min(min(a,b),c); }

__forceinline vdouble4 min(const vdouble4& a, const vdouble4&b, const vdouble4& c, const vdouble4& d) { return min(min(a,b),c,d); }


__forceinline vdouble4 max(const vdouble4& a, const vdouble4&b) { vdouble4 v;
                                                        v.f[0] = std::max(a.f[0],b.f[0]);
                                                        v.f[1] = std::max(a.f[1],b.f[1]);
                                                        v.f[2] = std::max(a.f[2],b.f[2]);
                                                        v.f[3] = std::max(a.f[3],b.f[3]);
							return v;
                                                       }

__forceinline vdouble4 max(const vdouble4& a, const vdouble4&b, const vdouble4& c) { return max(max(a,b),c); }

__forceinline vdouble4 max(const vdouble4& a, const vdouble4&b, const vdouble4& c, const vdouble4& d) { return max(max(a,b),c,d); }

__forceinline std::ostream& operator<<(std::ostream& cout, const vdouble4& a) {
   return cout << "<" << a[0] << ", " << a[1] << ", " << a[2] << ", " << a[3] << ">";
}

#endif
