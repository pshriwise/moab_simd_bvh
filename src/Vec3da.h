#pragma once

#include <assert.h>
#include <iostream>
#include <math.h>
#include "constants.h"
#include "Vec3ba.h"

struct Vec3da {
  typedef double Scalar;
  enum { N = 3 };
  double x,y,z;
  int a;

  inline Vec3da () {}

  inline Vec3da            ( const Vec3da& other ) { x = other.x; y = other.y; z = other.z; a = other.a; }
  inline Vec3da& operator =( const Vec3da& other ) { x = other.x; y = other.y; z = other.z; a = other.a; return *this;}

  inline Vec3da( const double pa ) { x = pa; y = pa; z = pa; a = pa;}
  inline Vec3da( const double pa[3]) { x = pa[0]; y = pa[1]; z = pa[2]; }
  inline Vec3da( const double px, const double py, const double pz) { x = px; y = py; z = pz; a = pz;}

  inline Vec3da( const double px, const double py, const double pz, const int pa) { x = px; y = py; z = pz; a = pa; }
    
  inline Vec3da( ZeroTy ) { x = 0.0f; y = 0.0f; z = 0.0f; a = 0;}
  inline Vec3da( PosInfTy ) { x = inf; y = inf; z = inf; a = inf; };
  inline Vec3da( NegInfTy ) { x = neg_inf; y = neg_inf; z = neg_inf; a = neg_inf; };

  inline const double& operator[](const size_t index) const { assert(index < 3); return (&x)[index]; }
  inline       double& operator[](const size_t index)       { assert(index < 3); return (&x)[index]; }

  inline double length () const { return sqrt(x*x + y*y + z*z); }

  inline Vec3da normalize() { double len = length();
    len = len < min_rcp_input ? min_rcp_input : len;
    x /= len; y /= len; z/= len; }
  
};


inline Vec3da operator +( const Vec3da& b, const Vec3da& c ) { return Vec3da(b.x+c.x, b.y+c.y, b.z+c.z, b.a+c.a); }
inline Vec3da operator -( const Vec3da& b, const Vec3da& c ) { return Vec3da(b.x-c.x, b.y-c.y, b.z-c.z, b.a-c.a); }
inline Vec3da operator *( const Vec3da& b, const Vec3da& c ) { return Vec3da(b.x*c.x, b.y*c.y, b.z*c.z, b.a*c.a); }
inline Vec3da operator *( const double& pa, const Vec3da& c ) { return Vec3da(pa) * c; }
inline Vec3da operator *( const Vec3da& c, const double& pa ) { return Vec3da(pa) * c; }
inline Vec3da operator /( const Vec3da& b, const Vec3da& c ) { return Vec3da(b.x/c.x, b.y/c.y, b.z/c.z, b.a/c.a); }
inline Vec3da operator /( const double& pa, const Vec3da& c ) { return Vec3da(pa) / c; }
inline Vec3da operator /( const Vec3da& c, const double& pa ) { return c / Vec3da(pa); }


inline bool operator ==( const Vec3da& b, const Vec3da& c) { return b.x == c.x &&
							            b.y == c.y &&
							            b.z == c.z;
                                                           }

inline const Vec3da min( const Vec3da& b, const Vec3da& c ) { return Vec3da(std::min(b.x,c.x),std::min(b.y,c.y),
									   std::min(b.z,c.z),std::min(b.a,c.a)); }
inline const Vec3da max( const Vec3da& b, const Vec3da& c ) { return Vec3da(std::max(b.x,c.x),std::max(b.y,c.y),
									   std::max(b.z,c.z),std::max(b.a,c.a)); }

inline const Vec3ba ge_mask( const Vec3da& b, const Vec3da& c ) { return Vec3ba(b.x >= c.x,b.y >= c.y,b.z >= c.z,b.a >= c.a); }
inline const Vec3ba le_mask( const Vec3da& b, const Vec3da& c ) { return Vec3ba(b.x <= c.x,b.y <= c.y,b.z <= c.z,b.a <= c.a); }

inline double reduce_add( const Vec3da &v ) { return v.x + v.y + v.z; }


inline double reduce_mul( const Vec3da& v ) { return v.x * v.y * v.z; }

inline double reduce_min( const Vec3da& v ) { return std::min(std::min(v.x, v.y), v.z); }

inline double reduce_max( const Vec3da& v ) { return std::max(std::max(v.x, v.y), v.z); }

inline double halfArea(Vec3da v) { return v.x*(v.y+v.z)+(v.y*v.z); }

inline Vec3da zero_fix( const Vec3da& a )
  {
    return Vec3da(fabs(a.x) < min_rcp_input ? double(min_rcp_input) : a.x,
                   fabs(a.y) < min_rcp_input ?  double(min_rcp_input) : a.y,
                   fabs(a.z) < min_rcp_input ? double(min_rcp_input) : a.z);
  }

inline const Vec3da rcp(const Vec3da& v ) { return Vec3da(1.0f/v.x,
							     1.0f/v.y,
							     1.0f/v.z); }

inline const Vec3da rcp_safe(const Vec3da& a) { return rcp(zero_fix(a)); }

inline Vec3da operator +( const Vec3da &a ) { return Vec3da(+a.x, +a.y, +a.z); }

inline Vec3da operator -( const Vec3da &a ) { return Vec3da(-a.x, -a.y, -a.z); }

inline double dot( const Vec3da& a, const Vec3da& b ) { return reduce_add(a*b); }



inline std::ostream& operator <<(std::ostream &os, Vec3da  const& v) {
  return os << '[' << v[0] << ' ' << v[1] << ' ' << v[2] << ' ' << v.a << ']';
}


