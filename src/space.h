#pragma once



#include "AABB.h"
#include "ops.h"
#include "Vec3vfa.h"


template<typename T>
struct LinSpaceT {

  typedef T Vector;
  typedef typename T::Scalar Scalar;
  
  
  /// CONSTRUCORS ///
  __forceinline LinSpaceT ( ) {}

  __forceinline LinSpaceT ( const LinSpaceT &other ) { vx = other.vx; vy = other.vy; vz = other.vz; }

  __forceinline LinSpaceT& operator=( const LinSpaceT &other ) { vx = other.vx; vy = other.vy; vz = other.vz; return *this; }

  __forceinline LinSpaceT ( const T &vx, const T &vy, const T& vz ) : vx(vx), vy(vy), vz(vz) {}

  __forceinline LinSpaceT ( const Scalar &val00, const Scalar &val01, const Scalar &val02,
			    const Scalar &val10, const Scalar &val11, const Scalar &val12,
			    const Scalar &val20, const Scalar &val21, const Scalar &val22 )
    : vx(val00, val10, val20), vy(val01, val11, val21), vz(val02, val12, val22) {}

  __forceinline LinSpaceT ( ZeroTy ) : vx(zero), vy(zero), vz(zero) {}
  
  
  /// LINEAR ALGEBRA OPERATIONS ///
  __forceinline const LinSpaceT transpose() const { return LinSpaceT(vx.x, vx.y, vx.z, vy.x, vy.y, vy.z, vz.x, vz.y, vz.z); }
  
  __forceinline const Scalar det() const { return dot(vx, cross(vy,vz)); }

  __forceinline const LinSpaceT adjoint() const { return LinSpaceT(cross(vy,vz), cross(vz,vx), cross(vx,vy)).transpose(); }

  __forceinline const LinSpaceT inverse() const { return adjoint()/det(); }

  /// ACCESS METHODS ///
  __forceinline const T row0() const { return T(vx.x, vy.x, vz.x); }
  __forceinline       T row0()       { return T(vx.x, vy.x, vz.x); }

  __forceinline const T row1() const { return T(vx.y, vy.y, vz.y); }
  __forceinline       T row1()       { return T(vx.y, vy.y, vz.y); }

  __forceinline const T row2() const { return T(vx.z, vy.z, vz.z); }
  __forceinline       T row2()       { return T(vx.z, vy.z, vz.z); }

  /// OPERATORS ///
  __forceinline       LinSpaceT operator /(const Scalar& v)       { return LinSpaceT(vx/v, vy/v, vz/v); }
  
  __forceinline const LinSpaceT operator /(const Scalar& v) const { return LinSpaceT(vx/v, vy/v, vz/v); }

  __forceinline LinSpaceT operator *=(const Scalar& v) { vx = vx * v; vy = vy * v; vz = vz * v; return *this; }

  __forceinline LinSpaceT operator /=(const Scalar& v) { vx = vx / v; vy = vy / v; vz = vz / v; return *this; }

  // create matrix for scaling
  static __forceinline LinSpaceT scale(const T &v) {
    return LinSpaceT(v.x, 0.0f, 0.0f,
		     0.0f, v.y, 0.0f,
		     0.0f, 0.0f, v.z);
  }

  
  /* column vectors of the matrix */
  T vx, vy, vz;
};



/// MORE OPERATORS ///
template<typename T>
__forceinline LinSpaceT<T> operator -(const LinSpaceT<T>& ls) { return LinSpaceT<T>(-ls.vx, -ls.vy, -ls.vz); }


template<typename T>
__forceinline LinSpaceT<T> operator +(const LinSpaceT<T>& ls) { return LinSpaceT<T>(+ls.vx, +ls.vy, +ls.vz); }
template<typename T>
__forceinline LinSpaceT<T> rcp       (const LinSpaceT<T>& ls) { return ls.inverse(); }

template<typename T>
__forceinline LinSpaceT<T> operator +( const LinSpaceT<T>& a, const LinSpaceT<T>& b) { return LinSpaceT<T>(a.vx+b.vx, a.vy+b.vy, a.vz+b.vz); }
template<typename T>
__forceinline LinSpaceT<T> operator -( const LinSpaceT<T>& a, const LinSpaceT<T>& b) { return LinSpaceT<T>(a.vx-b.vx, a.vy-b.vy, a.vz-b.vz); }

template<typename T>
__forceinline LinSpaceT<T> operator *( const typename T::Scalar& a, const LinSpaceT<T>& b) { return LinSpaceT<T>(a*b.vx, a*b.vy, a*b.vz); }


template<typename T>
__forceinline LinSpaceT<T> operator *( const float& a, const LinSpaceT<T>& b) { return LinSpaceT<T>(a*b.vx, a*b.vy, a*b.vz); }

template<typename T>
__forceinline T operator *( const LinSpaceT<T>& a, const T& b) { return madd(T(b.x),a.vx,madd(T(b.y),a.vy,T(b.z)*a.vz)); }

template<typename T>
__forceinline LinSpaceT<T> operator *( const LinSpaceT<T>& a, const LinSpaceT<T>& b) { return LinSpaceT<T>(a*b.vx, a*b.vy, a*b.vz); }


/// COMPARISON OPERATORS ///
template<typename T>
__forceinline bool operator ==(const LinSpaceT<T>& a, const LinSpaceT<T>& b) { return a.vx == b.vx && a.vy == b.vy && a.vz == b.vz; }

template<typename T>
__forceinline bool operator !=(const LinSpaceT<T>& a, const LinSpaceT<T>& b) { return a.vx != b.vx || a.vy != b.vy || a.vz != b.vz; }

/// SPATIAL OPERATORS ///


// rotation around an arbitrary axis in rads
template<typename T>
static __forceinline LinSpaceT<T> rotate(const T& v, const float& ang) {
  T u = v.normalized();
  float s = sin(ang), c = cos(ang);
  return LinSpaceT<T>(u.x*u.x+(1-u.x*u.x)*c,  u.x*u.y*(1-c)-u.z*s,    u.x*u.z*(1-c)+u.y*s,
		  u.x*u.y*(1-c)+u.z*s,    u.y*u.y+(1-u.y*u.y)*c,  u.y*u.z*(1-c)-u.x*s,
		  u.x*u.z*(1-c)-u.y*s,    u.y*u.z*(1-c)+u.x*s,    u.z*u.z+(1-u.z*u.z)*c);
}

template<typename T>
__forceinline LinSpaceT<T> transposed(const LinSpaceT<T>& ls) { return ls.transpose(); }

template<typename T>
__forceinline T xfmPnt(const LinSpaceT<T>& ls, const T& pnt) { return madd(T(pnt.x), ls.vx, madd(T(pnt.y),ls.vy,T(pnt.z)*ls.vz)); }

template<typename T>
__forceinline T xfmVector(const LinSpaceT<T>& ls, const T& vec) { return madd(T(vec.x), ls.vx, madd(T(vec.y),ls.vy,T(vec.z)*ls.vz)); }

template<typename T>
__forceinline T xfmNormal(const LinSpaceT<T>& ls, const T& norm) { return xfmVector(ls.inverse().transpose(), norm); }


/// OUTPUT OPERATOR ///
template<typename T>
static std::ostream& operator<<(std::ostream& cout, const LinSpaceT<T>& ls) {
  return cout << "{ vx = " << ls.vx << ", vy = " << ls.vy << ", vz = " << ls.vz << "}";
}

typedef LinSpaceT<Vec3fa> LinSpace;
typedef LinSpaceT<Vec3vfa> LinSpaceV;


#define V typename L::Vector

template< typename L >
struct AffineSpaceT {
  
  __forceinline AffineSpaceT () {}

  __forceinline AffineSpaceT ( const AffineSpaceT& other ) { l = other.l; p = other.p; }
  
  __forceinline AffineSpaceT& operator=( const AffineSpaceT& other ) { l = other.l; p = other.p; return *this; }
    
  __forceinline AffineSpaceT ( const Vec3vfa& vx, const Vec3vfa& vy, const Vec3vfa& vz, const Vec3vfa& p ) : l(vx,vy,vz) , p(p) {}

  __forceinline AffineSpaceT ( const L& l, const V& p ) : l(l), p(p) {}

  __forceinline AffineSpaceT ( const L& l ) : l(l), p(zero) {}

  __forceinline AffineSpaceT ( ZeroTy ) : l(zero), p(zero) {}
  
  /* __forceinline AffineSpaceT ( OneTy )  : l(one),  p(zero)  {} */

  static __forceinline const AffineSpaceT scale(const V& v) { return L::scale(v); }

  
  /* __forceinline AffineSpaceT scale(const Vec3vfa& s) { return L::scale(s); } */
  
  /* __forceinline AffineSpaceT rotat(const float&  r) { return L::rotate(r); } */
  
  L l;
  V p;
  
};

template<typename L>
__forceinline bool operator ==( const AffineSpaceT<L>& a, const AffineSpaceT<L>& b ) { return a.l == b.l && a.p == b.p; }
template<typename L>
__forceinline bool operator !=( const AffineSpaceT<L>& a, const AffineSpaceT<L>& b ) { return a.l != b.l || a.p != b.p; }

template<typename L>
__forceinline AffineSpaceT<L> operator -( const AffineSpaceT<L>& a ) { return AffineSpaceT<L>( -a.l, -a.p ); }

template<typename L>
__forceinline AffineSpaceT<L> operator +( const AffineSpaceT<L>& a ) { return AffineSpaceT<L>( +a.l, +a.p ); }

template<typename L>
__forceinline AffineSpaceT<L>        rcp( const AffineSpaceT<L>& a ) { L il = rcp(a.l); return AffineSpaceT<L>(il, -(il*a.p)); }

template<typename L>
__forceinline AffineSpaceT<L> operator +( const AffineSpaceT<L>& a, const AffineSpaceT<L>& b ) { return AffineSpaceT<L>(a.l + b.l, a.p + b.p); }
template<typename L>
__forceinline AffineSpaceT<L> operator -( const AffineSpaceT<L>& a, const AffineSpaceT<L>& b ) { return AffineSpaceT<L>(a.l - b.l, a.p - b.p); }

template<typename L>
__forceinline AffineSpaceT<L> operator *( const float&       a, const AffineSpaceT<L>& b ) { return AffineSpaceT<L>(a   * b.l, a   * b.p); }

template<typename L>
__forceinline AffineSpaceT<L> operator *( const AffineSpaceT<L>& a, const float&       b) { return AffineSpaceT<L>(b   * a.l, b   * a.p); }

template<typename L>
__forceinline AffineSpaceT<L> operator *( const AffineSpaceT<L>& a, const AffineSpaceT<L>& b ) { return AffineSpaceT<L>(a.l * b.l, a.p * b.p); }

template<typename L>
__forceinline AffineSpaceT<L> operator /( const AffineSpaceT<L>& a, const AffineSpaceT<L>& b ) { return a * rcp(b); }

template<typename L>
__forceinline AffineSpaceT<L> operator /( const AffineSpaceT<L>& a, const float&       b ) { return a * rcp(b); }

template<typename L>
__forceinline AffineSpaceT<L>& operator *=( AffineSpaceT<L>& a, const AffineSpaceT<L>& b ) { return a = a * b; }
template<typename L>
__forceinline AffineSpaceT<L>& operator *=( AffineSpaceT<L>& a, const float&       b ) { return a = a * b; }
template<typename L>
__forceinline AffineSpaceT<L>& operator /=( AffineSpaceT<L>& a, const AffineSpaceT<L>& b ) { return a = a / b; }
template<typename L>
__forceinline AffineSpaceT<L>& operator /=( AffineSpaceT<L>& a, const float&       b ) { return a = a / b; }

template<typename L>
__forceinline const V xfmPoint (const AffineSpaceT<L>& m, const V& p) { return madd(V(p.x),m.l.vx,madd(V(p.y),m.l.vy,madd(V(p.z),m.l.vz,m.p))); }
template<typename L>
__forceinline const V xfmVector(const AffineSpaceT<L>& m, const V& v) { return xfmVector(m.l,v); }
template<typename L>
__forceinline const V xfmNormal(const AffineSpaceT<L>& m, const Vec3vfa& n) { return xfmNormal(m.l,n); }

template<typename L>
__forceinline const AABB xfmBounds(const L& l, const AABB& b) {
  AABB dst = AABB(zero);
  const Vec3fa p0(b.lower.x, b.lower.y, b.lower.z); dst.update(xfmPnt(l, p0));
  const Vec3fa p1(b.lower.x, b.lower.y, b.upper.z); dst.update(xfmPnt(l, p1));
  const Vec3fa p2(b.lower.x, b.upper.y, b.lower.z); dst.update(xfmPnt(l, p2));
  const Vec3fa p3(b.lower.x, b.upper.y, b.upper.z); dst.update(xfmPnt(l, p3));
  const Vec3fa p4(b.upper.x, b.lower.y, b.lower.z); dst.update(xfmPnt(l, p4));
  const Vec3fa p5(b.upper.x, b.lower.y, b.upper.z); dst.update(xfmPnt(l, p5));
  const Vec3fa p6(b.upper.x, b.upper.y, b.lower.z); dst.update(xfmPnt(l, p6));
  const Vec3fa p7(b.upper.x, b.upper.y, b.upper.z); dst.update(xfmPnt(l, p7));
  return dst;
}


#undef V

template<typename L>
static std::ostream& operator<<(std::ostream& cout, const AffineSpaceT<L>& m){
  return cout << "{ l = " << m.l << ", p = " << m.p << " }";
}

typedef AffineSpaceT<LinSpace> AffineSpace3fa;
typedef AffineSpaceT<LinSpaceV> AffineSpace;
