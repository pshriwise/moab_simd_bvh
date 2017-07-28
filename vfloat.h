#pragma once


#include "constants.h"
#include "Vec3.h"

struct vfloat4
{

  enum { size = 4 }; // number of elements
  float f[4]; // data holder

  inline vfloat4 () {}
  inline vfloat4 (const vfloat4& other) { f[0] = other.f[0];
                                          f[1] = other.f[1];
					  f[2] = other.f[2];
					  f[3] = other.f[2]; }

  inline vfloat4( float a ) { f[0] = a; f[1] = a; f[2] = a; f[3] = a; }
  inline vfloat4( float a, float b, float c, float d) {  f[0] = a; f[1] = b; f[2] = c; f[3] = d; }

  inline vfloat4 load ( const void* const a) { for(size_t i=0; i < 4; i++) f[i] = *(float*)((const char*)a+i*sizeof(float)); }
  //[0] = (float*)a[0]; f[1] = (float*)a[1]; f[2] = (float*)a[2]; f[3] = (float*)a[3]; }

  inline const float& operator [](const size_t index) const { assert(index < 4); return f[index]; }
  inline       float& operator [](const size_t index)       { assert(index < 4); return f[index]; }

  

};
  
  
