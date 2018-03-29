#pragma once



#include "AABB.h"
#include "ops.h"

struct LinSpace {


  /// CONSTRUCORS ///
  __forceinline LinSpace ( ) {}

  __forceinline LinSpace ( const LinSpace &other ) { vx = other.vx; vy = other.vy; vz = other.vz; }

  __forceinline LinSpace& operator=( const LinSpace &other ) { vx = other.vx; vy = other.vy; vz = other.vz; return *this; }

  __forceinline LinSpace ( const Vec3fa &vx, const Vec3fa &vy, const Vec3fa& vz ) : vx(vx), vy(vy), vz(vz) {}

  __forceinline LinSpace ( const float &val00, const float &val01, const float &val02,
			   const float &val10, const float &val11, const float &val12,
			   const float &val20, const float &val21, const float &val22 )
    : vx(val00, val10, val20), vy(val01, val11, val21), vz(val02, val12, val22) {}

  __forceinline LinSpace (const float& val) : vx(val), vy(val), vz(val) {}
  
  /// LINEAR ALGEBRA OPERATIONS ///
  __forceinline const LinSpace transpose() const { return LinSpace(vx.x, vx.y, vx.z, vy.x, vy.y, vy.z, vz.x, vz.y, vz.z); }
  
  __forceinline const float det() const { return dot(vx, cross(vy,vz)); }

  __forceinline const LinSpace adjoint() const { return LinSpace(cross(vy,vz), cross(vz,vx), cross(vx,vy)).transpose(); }

  __forceinline const LinSpace inverse() const { return adjoint()/det(); }

  /// ACCESS METHODS ///
  __forceinline const Vec3fa row0() const { return Vec3fa(vx.x, vy.x, vz.x); }
  __forceinline       Vec3fa row0()       { return Vec3fa(vx.x, vy.x, vz.x); }

  __forceinline const Vec3fa row1() const { return Vec3fa(vx.y, vy.y, vz.y); }
  __forceinline Vec3fa row1() { return Vec3fa(vx.y, vy.y, vz.y); }

  __forceinline const Vec3fa row2() const { return Vec3fa(vx.z, vy.z, vz.z); }
  __forceinline       Vec3fa row2()       { return Vec3fa(vx.z, vy.z, vz.z); }

  /// OPERATORS ///
  __forceinline       LinSpace operator /(const float& v)       { return LinSpace(vx/v, vy/v, vz/v); }
  
  __forceinline const LinSpace operator /(const float& v) const { return LinSpace(vx/v, vy/v, vz/v); }

  __forceinline LinSpace operator *=(const float& v) { vx = vx * v; vy = vy * v; vz = vz * v; return *this; }

  __forceinline LinSpace operator /=(const float& v) { vx = vx / v; vy = vy / v; vz = vz / v; return *this; }

  /* column vectors of the matrix */
  Vec3fa vx, vy, vz;
};

/// MORE OPERATORS ///
__forceinline LinSpace operator -(const LinSpace& ls) { return LinSpace(-ls.vx, -ls.vy, -ls.vz); }
__forceinline LinSpace operator +(const LinSpace& ls) { return LinSpace(+ls.vx, +ls.vy, +ls.vz); }
__forceinline LinSpace rcp       (const LinSpace& ls) { return ls.inverse(); }

__forceinline LinSpace operator +( const LinSpace& a, const LinSpace& b) { return LinSpace(a.vx+b.vx, a.vy+b.vy, a.vz+b.vz); }
__forceinline LinSpace operator -( const LinSpace& a, const LinSpace& b) { return LinSpace(a.vx-b.vx, a.vy-b.vy, a.vz-b.vz); }

__forceinline LinSpace operator *( float& a, const LinSpace& b) { return LinSpace(a*b.vx, a*b.vy, a*b.vz); }

__forceinline Vec3fa operator *( const LinSpace& a, const Vec3fa& b) { return madd(Vec3fa(b.x),a.vx,madd(Vec3fa(b.y),a.vy,Vec3fa(b.z)*a.vz)); }

__forceinline LinSpace operator *( const LinSpace& a, const LinSpace& b) { return LinSpace(a*b.vx, a*b.vy, a*b.vz); }


/// COMPARISON OPERATORS ///
__forceinline bool operator ==(const LinSpace& a, const LinSpace& b) { return a.vx == b.vx && a.vy == b.vy && a.vz == b.vz; }

__forceinline bool operator !=(const LinSpace& a, const LinSpace& b) { return a.vx != b.vx || a.vy != b.vy || a.vz != b.vz; }

/// SPATIAL OPERATORS ///

// create matrix for scaling
static __forceinline LinSpace scale(const Vec3fa &v) {
  return LinSpace(v.x, 0.0f, 0.0f,
		  0.0f, v.y, 0.0f,
		  0.0f, 0.0f, v.z);
}


// rotation around an arbitrary axis in rads
static __forceinline LinSpace rotate(const Vec3fa& v, const float& ang) {
  Vec3fa u = v.normalized();
  float s = sin(ang), c = cos(ang);
  return LinSpace(u.x*u.x+(1-u.x*u.x)*c,  u.x*u.y*(1-c)-u.z*s,    u.x*u.z*(1-c)+u.y*s,
		  u.x*u.y*(1-c)+u.z*s,    u.y*u.y+(1-u.y*u.y)*c,  u.y*u.z*(1-c)-u.x*s,
		  u.x*u.z*(1-c)-u.y*s,    u.y*u.z*(1-c)+u.x*s,    u.z*u.z+(1-u.z*u.z)*c);
}

__forceinline LinSpace transposed(const LinSpace& ls) { return ls.transpose(); }

__forceinline Vec3fa xfmPnt(const LinSpace& ls, const Vec3fa& pnt) { return madd(Vec3fa(pnt.x), ls.vx, madd(Vec3fa(pnt.y),ls.vy,Vec3fa(pnt.z)*ls.vz)); }

__forceinline Vec3fa xfmVector(const LinSpace& ls, const Vec3fa& vec) { return madd(Vec3fa(vec.x), ls.vx, madd(Vec3fa(vec.y),ls.vy,Vec3fa(vec.z)*ls.vz)); }

__forceinline Vec3fa xfmNormal(const LinSpace& ls, const Vec3fa& norm) { return xfmVector(ls.inverse().transpose(), norm); }


/// OUTPUT OPERATOR ///

static std::ostream& operator<<(std::ostream& cout, const LinSpace& ls) {
  return cout << "{ vx = " << ls.vx << ", vy = " << ls.vy << ", vz = " << ls.vz << "}";
}


struct AffineSpace {

  __forceinline AffineSpace () {}

  __forceinline AffineSpace ( const AffineSpace& other ) { l = other.l; p = other.p; }
  
  __forceinline AffineSpace& operator=( const AffineSpace& other ) { l = other.l; p = other.p; return *this; }
    
  __forceinline AffineSpace ( const Vec3fa& vx, const Vec3fa& vy, const Vec3fa& vz, const Vec3fa& p ) : l(vx,vy,vz) , p(p) {}

  __forceinline AffineSpace ( const LinSpace& l, const Vec3fa& p ) : l(l), p(p) {}

  __forceinline AffineSpace ( ZeroTy ) : l(zero), p(zero) {}
  
  /* __forceinline AffineSpace ( OneTy )  : l(one),  p(zero)  {} */

  /* __forceinline AffineSpace scale(const Vec3fa& s) { return LinSpace::scale(s); } */
  
  /* __forceinline AffineSpace rotat(const float&  r) { return LinSpace::rotate(r); } */
  
  LinSpace l;
  Vec3fa p;
  
};

__forceinline bool operator ==( const AffineSpace& a, const AffineSpace& b ) { return a.l == b.l && a.p == b.p; }
__forceinline bool operator !=( const AffineSpace& a, const AffineSpace& b ) { return a.l != b.l || a.p != b.p; }

__forceinline AffineSpace operator -( const AffineSpace& a ) { return AffineSpace( -a.l, -a.p ); }
__forceinline AffineSpace operator +( const AffineSpace& a ) { return AffineSpace( +a.l, +a.p ); }
__forceinline AffineSpace        rcp( const AffineSpace& a ) { LinSpace il = rcp(a.l); return AffineSpace(il, -(il*a.p)); }

__forceinline AffineSpace operator +( const AffineSpace& a, const AffineSpace& b ) { return AffineSpace(a.l + b.l, a.p + b.p); }
__forceinline AffineSpace operator -( const AffineSpace& a, const AffineSpace& b ) { return AffineSpace(a.l - b.l, a.p - b.p); }

__forceinline AffineSpace operator *( const float&       a, const AffineSpace& b ) { return AffineSpace(a   * b.l, a   * b.p); }
__forceinline AffineSpace operator *( const AffineSpace& a, const float& b )       { return AffineSpace(b   * a.l, b   * a.p); }

__forceinline AffineSpace operator *( const AffineSpace& a, const AffineSpace& b ) { return AffineSpace(a.l * b.l, a.p * b.p); }

__forceinline AffineSpace operator /( const AffineSpace& a, const AffineSpace& b ) { return a * rcp(b); }
__forceinline AffineSpace operator /( const AffineSpace& a, const float&       b ) { return a * rcp(b); }

__forceinline AffineSpace& operator *=( AffineSpace& a, const AffineSpace& b ) { return a = a * b; }
__forceinline AffineSpace& operator *=( AffineSpace& a, const float&       b ) { return a = a * b; }
__forceinline AffineSpace& operator /=( AffineSpace& a, const AffineSpace& b ) { return a = a / b; }
__forceinline AffineSpace& operator /=( AffineSpace& a, const float&       b ) { return a = a / b; }

__forceinline const Vec3fa xfmPoint (const AffineSpace& m, const Vec3fa& p) { return madd(Vec3fa(p.x),m.l.vx,madd(Vec3fa(p.y),m.l.vy,madd(Vec3fa(p.z),m.l.vz,m.p))); }
__forceinline const Vec3fa xfmVector(const AffineSpace& m, const Vec3fa& v) { return xfmVector(m.l,v); }
__forceinline const Vec3fa xfmNormal(const AffineSpace& m, const Vec3fa& n) { return xfmNormal(m.l,n); }

__forceinline const AABB xfmBounds(const AffineSpace m, AABB b) {
  AABB dst = AABB(zero);
  const Vec3fa p0(b.lower.x, b.lower.y, b.lower.z); dst.update(xfmPoint(m, p0));
  const Vec3fa p1(b.lower.x, b.lower.y, b.upper.z); dst.update(xfmPoint(m, p1));
  const Vec3fa p2(b.lower.x, b.upper.y, b.lower.z); dst.update(xfmPoint(m, p2));
  const Vec3fa p3(b.lower.x, b.upper.y, b.upper.z); dst.update(xfmPoint(m, p3));
  const Vec3fa p4(b.upper.x, b.lower.y, b.lower.z); dst.update(xfmPoint(m, p4));
  const Vec3fa p5(b.upper.x, b.lower.y, b.upper.z); dst.update(xfmPoint(m, p5));
  const Vec3fa p6(b.upper.x, b.upper.y, b.lower.z); dst.update(xfmPoint(m, p6));
  const Vec3fa p7(b.upper.x, b.upper.y, b.upper.z); dst.update(xfmPoint(m, p7));
  return dst;
}

static std::ostream& operator<<(std::ostream& cout, const AffineSpace& m){
  return cout << "{ l = " << m.l << ", p = " << m.p << " }";
}
