#pragma once


#include "Vec3fa.h"

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

__forceinline Vec3fa xfmVec(const LinSpace& ls, const Vec3fa& vec) { return madd(Vec3fa(vec.x), ls.vx, madd(Vec3fa(vec.y),ls.vy,Vec3fa(vec.z)*ls.vz)); }

__forceinline Vec3fa xfmNorm(const LinSpace& ls, const Vec3fa& norm) { return xfmVec(ls.inverse().transpose(), norm); }


/// OUTPUT OPERATOR ///

static std::ostream& operator<<(std::ostream& cout, const LinSpace& ls) {
  return cout << "{ vx = " << ls.vx << ", vy = " << ls.vy << ", vz = " << ls.vz << "}";
}
