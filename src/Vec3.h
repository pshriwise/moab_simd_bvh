
#pragma once

#include <assert.h>
#include <iostream>
#include <math.h>
#include "constants.h"


template<typename T> struct Vec3 {

  T x,y,z;
  
  inline Vec3() {}
    
  inline Vec3( const Vec3 &other) { x = other.x;
                                    y = other.y;
				    z = other.z; }
  
  /* inline Vec3( const Vec3fa& other) { x = other.x; */
  /*                                     y = other.y; */
  /* 				      z = other.z; ) */
  
  inline Vec3& operator =( const Vec3& other ) { x = other.x;
						 y = other.y;
						 z = other.z;
						 return *this; }

  /* inline Vec3& operator =( const Vec3fa& other ) { x = other.x; */
  /* 						 y = other.y; */
  /* 						 z = other.z; */
  /* 						 return *this; } */


  inline const T& operator []( const size_t axis) const { assert(axis < 3); return (&x)[axis]; }
  inline       T& operator []( const size_t axis)       { assert(axis < 3); return (&x)[axis]; }

  inline void normalize() { T len = length();
                            len = len < min_rcp_input ? min_rcp_input : len;
                            x /= len; y /= len; z /= len; }



  inline Vec3( const T x, const T y, const T z) : x(x), y(y), z(z) {}
  
  inline Vec3( const T v[3] ) : x(v[0]), y(v[1]), z(v[2]) {}
  
  inline T length() const { return sqrtf(x*x + y*y + z*z); }
  
  template<typename t>
  friend std::ostream& operator <<(std::ostream &os, Vec3<t> const&v);

};


template<typename T>
inline Vec3<T> operator +( const Vec3<T> &a ) { return Vec3<T>(+a.x, +a.y, +a.z); }
template<typename T>
inline Vec3<T> operator -( const Vec3<T> &a ) { return Vec3<T>(-a.x, -a.y, -a.z); }

template<typename T>
inline Vec3<T> operator +( const Vec3<T> &a, const Vec3<T> &b ) { return Vec3<T>(a.x+b.x, a.y+b.y, a.z+b.z); }
template<typename T>
inline Vec3<T> operator -( const Vec3<T> &a, const Vec3<T> &b ) { return Vec3<T>(a.x-b.x, a.y-b.y, a.z-b.z); }

template<typename T>
inline Vec3<T> operator *( const Vec3<T>& a, const Vec3<T>& b ) { return Vec3<T>(a.x * b.x, a.y * b.y, a.z * b.z); }
template<typename T>
inline Vec3<T> operator *( const       T& a, const Vec3<T>& b ) { return Vec3<T>(a   * b.x, a   * b.y, a   * b.z); }
template<typename T>
inline Vec3<T> operator *( const Vec3<T>& a, const       T& b ) { return Vec3<T>(a.x * b  , a.y * b  , a.z * b  ); }
template<typename T>
inline Vec3<T> operator /( const Vec3<T>& a, const       T& b ) { return Vec3<T>(a.x / b  , a.y / b  , a.z / b  ); }
template<typename T>
inline Vec3<T> operator /( const       T& a, const Vec3<T>& b ) { return Vec3<T>(a   / b.x, a   / b.y, a   / b.z); }
template<typename T>
inline Vec3<T> operator /( const Vec3<T>& a, const Vec3<T>& b ) { return Vec3<T>(a.x / b.x, a.y / b.y, a.z / b.z); }


template<typename T>
inline Vec3<bool> ge_mask( const Vec3<T> &a, const Vec3<T> &b ) { return Vec3<bool>(a.x >= b.x,a.y >= b.y, a.z >= b.z); }

template<typename T>
inline Vec3<bool> le_mask( const Vec3<T> &a, const Vec3<T> &b) { return Vec3<bool>(a.x <= b.x,a.y <= b.y, a.z <= b.z); }

template<typename T>
inline Vec3<T> min(const Vec3<T>& a, const Vec3<T>& b) { return Vec3<T>(std::min(a.x,b.x), std::min(a.y,b.y), std::min(a.z,b.z)); }

template<typename T>
inline Vec3<T> max(const Vec3<T>& a, const Vec3<T>& b) { return Vec3<T>(std::max(a.x,b.x), std::max(a.y,b.y), std::max(a.z,b.z)); }

template<typename T>
inline T reduce_add( const Vec3<T> &v ) { return v.x + v.y + v.z; }

template<typename T>
inline T reduce_mul( const Vec3<T>& v ) { return v.x * v.y * v.z; }

template<typename T>
inline T reduce_min( const Vec3<T>& v ) { return std::min(std::min(v.x, v.y), v.z); }

template<typename T>
inline T reduce_max( const Vec3<T>& v ) { return std::max(std::max(v.x, v.y), v.z); }

inline bool all(bool b[3]) { return b[0] && b[1] && b[2]; }

typedef Vec3<float> Vec3f;

template<typename T>
inline std::ostream& operator <<(std::ostream &os, Vec3<T> const& v) {
  return os << '[' << v[0] << ' ' << v[1] << ' ' << v[2] << ']';
}

template<typename T>
inline bool all(Vec3<T> v) { return v[0] && (v[0] == v[1]) && (v[0] == v[2]); }

template<typename T>
inline T halfArea(Vec3<T> v) { return v.x*(v.y+v.z)+(v.y*v.z); }

template<typename T>
inline Vec3<T> zero_fix( const Vec3<T>& a )
  {
    return Vec3<T>(fabs(a.x) < min_rcp_input ? T(min_rcp_input) : a.x,
                   fabs(a.y) < min_rcp_input ?  T(min_rcp_input) : a.y,
                   fabs(a.z) < min_rcp_input ? T(min_rcp_input) : a.z);
  }

/* template<typename T> */
/* inline const Vec3<T> rcp(const Vec3<T>& v ) { return Vec3<T>(v.x == 0 ? inf : 1.0f/v.x, */
/* 						v.y == 0 ? inf : 1.0f/v.y, */
/* 						v.z == 0 ? inf : 1.0f/v.z); } */

template<typename T>
inline const Vec3<T> rcp(const Vec3<T>& v ) { return Vec3<T>(1.0f/v.x,
							     1.0f/v.y,
							     1.0f/v.z); }

template<typename T>
inline const Vec3<T> rcp_safe(const Vec3<T>& a) { return rcp(zero_fix(a)); }


