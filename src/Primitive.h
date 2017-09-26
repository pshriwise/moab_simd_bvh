

#pragma once

#include "AABB.h"

struct BuildPrimitive {
  float lower_x, lower_y, lower_z;
  int sceneID;
  float upper_x, upper_y, upper_z;
  int primID;

  BuildPrimitive () {}
  
  BuildPrimitive (float lx, float ly, float lz, int sID, float ux, float uy, float uz, int pID) :
  lower_x(lx), lower_y(ly), lower_z(lz), sceneID(sID), upper_x(ux), upper_y(uy), upper_z(uz), primID(pID) {}
  
  friend bool operator< (const BuildPrimitive& a, const BuildPrimitive& b) { return a.primID < b.primID; }

  friend bool operator!= (const BuildPrimitive& a, const BuildPrimitive& b) { return a.primID != b.primID; }

  friend bool operator== (const BuildPrimitive& a, const BuildPrimitive& b) { return a.primID == b.primID; }

  AABB box() { return AABB(lower_x, lower_y, lower_z, upper_x, upper_y, upper_z); }
  
  Vec3fa center() const { return Vec3fa(lower_x+upper_x,lower_y+upper_y,lower_z+upper_z)/2.0f; }

  inline bool intersect(const TravRay &ray, float& nearest_hit) {

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

    const float tnearx = (xnear - ray.org.x) * ray.rdir.x;
    const float tfarx = (xfar - ray.org.x) * ray.rdir.x;
  
    const float tneary = (ynear - ray.org.y) * ray.rdir.y;
    const float tfary = (yfar - ray.org.y) * ray.rdir.y;

    const float tnearz = (znear - ray.org.z) * ray.rdir.z;
    const float tfarz = (zfar - ray.org.z) * ray.rdir.z;

    const float round_down = 1.0f-2.0f*float(ulp); // FIXME: use per instruction rounding for AVX512
    const float round_up   = 1.0f+2.0f*float(ulp);

    const float tmin = std::max(tnearx,std::max(tneary,tnearz));
    const float tmax = std::min(tfarx,std::min(tfary,tfarz));

    if (tmax > tmin && tmax >= 0) {
      nearest_hit = tmin >= 0 ? (tmin*ray.dir).length() : 0;
      return true;
    }
    else {
      nearest_hit = inf;
      return false;
    }
    
  }

};


inline std::ostream& operator<< (std::ostream &os, BuildPrimitive &p) {
  return os <<  "Primitive "  << p.primID << std::endl <<
                "Lower xyz: " << p.lower_x << " "
	                      << p.lower_y << " "
                              << p.lower_z << std::endl <<
                "Upper xyz: " << p.upper_x << " "
                              << p.upper_y << " "
                              << p.upper_z << std::endl <<
                "Part of Scene " << p.sceneID << std::endl;
}

