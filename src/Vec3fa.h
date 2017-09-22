#pragma once

#include <assert.h>
#include <iostream>
#include <math.h>
#include "constants.h"
#include "Vec3ba.h"

struct Vec3fa {
  typedef float Scalar;
  enum { N = 3 };
  float x,y,z;
  int a;

  inline Vec3fa () {}

  inline Vec3fa            ( const Vec3fa& other ) { x = other.x; y = other.y; z = other.z; a = other.a; }
  inline Vec3fa& operator =( const Vec3fa& other ) { x = other.x; y = other.y; z = other.z; a = other.a; return *this;}

  inline Vec3fa( const float pa ) { x = pa; y = pa; z = pa; a = pa;}
  inline Vec3fa( const float pa[3]) { x = pa[0]; y = pa[1]; z = pa[2]; }
  inline Vec3fa( const float px, const float py, const float pz) { x = px; y = py; z = pz; a = pz;}

  inline Vec3fa( const float px, const float py, const float pz, const int pa) { x = px; y = py; z = pz; a = pa; }
    
  inline Vec3fa( ZeroTy ) { x = 0.0f; y = 0.0f; z = 0.0f; a = 0;}
  inline Vec3fa( PosInfTy ) { x = inf; y = inf; z = inf; a = inf; };
  inline Vec3fa( NegInfTy ) { x = neg_inf; y = neg_inf; z = neg_inf; a = neg_inf; };

  inline const float& operator[](const size_t index) const { assert(index < 3); return (&x)[index]; }
  inline       float& operator[](const size_t index)       { assert(index < 3); return (&x)[index]; }

  inline float length () const { return sqrtf(x*x + y*y + z*z); }

  inline Vec3fa normalize() { float len = length();
    len = len < min_rcp_input ? min_rcp_input : len;
    x /= len; y /= len; z/= len; }
  
};


inline Vec3fa operator +( const Vec3fa& b, const Vec3fa& c ) { return Vec3fa(b.x+c.x, b.y+c.y, b.z+c.z, b.a+c.a); }
inline Vec3fa operator -( const Vec3fa& b, const Vec3fa& c ) { return Vec3fa(b.x-c.x, b.y-c.y, b.z-c.z, b.a-c.a); }
inline Vec3fa operator *( const Vec3fa& b, const Vec3fa& c ) { return Vec3fa(b.x*c.x, b.y*c.y, b.z*c.z, b.a*c.a); }
inline Vec3fa operator *( const float& pa, const Vec3fa& c ) { return Vec3fa(pa) * c; }
inline Vec3fa operator *( const Vec3fa& c, const float& pa ) { return Vec3fa(pa) * c; }
inline Vec3fa operator /( const Vec3fa& b, const Vec3fa& c ) { return Vec3fa(b.x/c.x, b.y/c.y, b.z/c.z, b.a/c.a); }
inline Vec3fa operator /( const float& pa, const Vec3fa& c ) { return Vec3fa(pa) / c; }
inline Vec3fa operator /( const Vec3fa& c, const float& pa ) { return c / Vec3fa(pa); }


inline bool operator ==( const Vec3fa& b, const Vec3fa& c) { return b.x == c.x &&
							            b.y == c.y &&
							            b.z == c.z;
                                                           }

inline const Vec3fa min( const Vec3fa& b, const Vec3fa& c ) { return Vec3fa(std::min(b.x,c.x),std::min(b.y,c.y),
									   std::min(b.z,c.z),std::min(b.a,c.a)); }
inline const Vec3fa max( const Vec3fa& b, const Vec3fa& c ) { return Vec3fa(std::max(b.x,c.x),std::max(b.y,c.y),
									   std::max(b.z,c.z),std::max(b.a,c.a)); }

inline const Vec3ba ge_mask( const Vec3fa& b, const Vec3fa& c ) { return Vec3ba(b.x >= c.x,b.y >= c.y,b.z >= c.z,b.a >= c.a); }
inline const Vec3ba le_mask( const Vec3fa& b, const Vec3fa& c ) { return Vec3ba(b.x <= c.x,b.y <= c.y,b.z <= c.z,b.a <= c.a); }

inline float reduce_add( const Vec3fa &v ) { return v.x + v.y + v.z; }


inline float reduce_mul( const Vec3fa& v ) { return v.x * v.y * v.z; }

inline float reduce_min( const Vec3fa& v ) { return std::min(std::min(v.x, v.y), v.z); }

inline float reduce_max( const Vec3fa& v ) { return std::max(std::max(v.x, v.y), v.z); }

inline float halfArea(Vec3fa v) { return v.x*(v.y+v.z)+(v.y*v.z); }

inline Vec3fa zero_fix( const Vec3fa& a )
  {
    return Vec3fa(fabs(a.x) < min_rcp_input ? float(min_rcp_input) : a.x,
                   fabs(a.y) < min_rcp_input ?  float(min_rcp_input) : a.y,
                   fabs(a.z) < min_rcp_input ? float(min_rcp_input) : a.z);
  }

inline const Vec3fa rcp(const Vec3fa& v ) { return Vec3fa(1.0f/v.x,
							     1.0f/v.y,
							     1.0f/v.z); }

inline const Vec3fa rcp_safe(const Vec3fa& a) { return rcp(zero_fix(a)); }

inline Vec3fa operator +( const Vec3fa &a ) { return Vec3fa(+a.x, +a.y, +a.z); }

inline Vec3fa operator -( const Vec3fa &a ) { return Vec3fa(-a.x, -a.y, -a.z); }

inline float dot( const Vec3fa& a, const Vec3fa& b ) { return reduce_add(a*b); }



inline std::ostream& operator <<(std::ostream &os, Vec3fa  const& v) {
  return os << '[' << v[0] << ' ' << v[1] << ' ' << v[2] << ' ' << v.a << ']';
}


