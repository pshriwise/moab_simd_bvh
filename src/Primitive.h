

#pragma once

#include "AABB.h"

struct BuildPrimitive {
  Vec3fa lower, upper;
  /* float lower_x, lower_y, lower_z; */
  /* int sceneID; */
  /* float upper_x, upper_y, upper_z; */
  /* int primID; */

  BuildPrimitive () {}
  
  BuildPrimitive (float lx, float ly, float lz, int sID, float ux, float uy, float uz, int pID) : lower(Vec3fa(lx, ly, lz, sID)),
    upper(Vec3fa(ux, uy, uz, pID)) {}
  
  /* int sceneID() { return lower.a; } */

  /* int primID() { return upper.a; } */

  const int sceneID() const { return lower.a; }

  const int primID() const { return upper.a; }

  void set_sceneID( const int &id ) { lower.a = id; }

  void set_primID( const int &id ) { upper.a = id; }

  friend bool operator< (const BuildPrimitive& a, const BuildPrimitive& b) { return a.primID() < b.primID(); }

  friend bool operator!= (const BuildPrimitive& a, const BuildPrimitive& b) { return a.primID() != b.primID(); }

  friend bool operator== (const BuildPrimitive& a, const BuildPrimitive& b) { return a.primID() == b.primID(); }

  AABB box() { return AABB(lower.x, lower.y, lower.z, upper.x, upper.y, upper.z); }
  
  Vec3fa center() const { return Vec3fa(lower.x+upper.x,lower.y+upper.y,lower.z+upper.z)/2.0f; }

  template<typename R>
  inline bool intersect(R &ray) {

    AABB b = box();
  
    float xnear, xfar, ynear, yfar, znear, zfar;
  
    if (ray.dir.x >= 0.0f) {
      xnear = b.lower.x;
      xfar = b.upper.x;
    }
    else {
      xnear = b.upper.x;
      xfar = b.lower.x;
    }

    if (ray.dir.y >= 0.0f) {
      ynear = b.lower.y;
      yfar = b.upper.y;
    }
    else {
      ynear = b.upper.y;
      yfar = b.lower.y;
    }    

    if (ray.dir.z >= 0.0f) {
      znear = b.lower.z;
      zfar = b.upper.z;
    }
    else {
      znear = b.upper.z;
      zfar = b.lower.z;
    }    

    const float tnearx = (xnear - ray.org.x) * (1/ray.dir.x);
    const float tfarx = (xfar - ray.org.x) * (1/ray.dir.x);
  
    const float tneary = (ynear - ray.org.y) * (1/ray.dir.y);
    const float tfary = (yfar - ray.org.y) * (1/ray.dir.y);

    const float tnearz = (znear - ray.org.z) * (1/ray.dir.z);
    const float tfarz = (zfar - ray.org.z) * (1/ray.dir.z);

    const float round_down = 1.0f-2.0f*float(ulp); // FIXME: use per instruction rounding for AVX512
    const float round_up   = 1.0f+2.0f*float(ulp);

    const float tmin = std::max(tnearx,std::max(tneary,tnearz));
    const float tmax = std::min(tfarx,std::min(tfary,tfarz));

    float nearest_hit;
    
    if (tmax > tmin && tmax >= 0) {
      nearest_hit = tmin >= 0 ? (tmin*ray.dir).length() : 0;
      if (nearest_hit < ray.tfar) {
	ray.tfar = nearest_hit;
	ray.primID = primID();
      }
      return true;
    }
    else {
      return false;
    }
  }

};


inline std::ostream& operator<< (std::ostream &os, BuildPrimitive &p) {
  return os <<  "Primitive "  << p.primID() << std::endl <<
                "Lower xyz: " << p.lower.x << " "
	                      << p.lower.y << " "
                              << p.lower.z << std::endl <<
                "Upper xyz: " << p.upper.x << " "
                              << p.upper.y << " "
                              << p.upper.z << std::endl <<
                "Part of Scene " << p.sceneID() << std::endl;
}

