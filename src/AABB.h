
#pragma once

#include <cstring>
#include <algorithm>


#include "Vec3.h"
#include "Ray.h"
#include "constants.h"
#include "Vec3fa.h"

struct AABB {

  Vec3fa lower, upper;

  inline AABB() {}

  inline AABB ( float x_min, float y_min, float z_min, float x_max, float y_max, float z_max) { lower[0] = x_min;
                                                                                               lower[1] = y_min;
											       lower[2] = z_min;
											       upper[0] = x_max;
											       upper[1] = y_max;
											       upper[2] = z_max; }

  inline AABB( const float low, const float high) { lower = low; upper = high; }
  
  
  inline AABB ( const float ext[6] )                                                          { lower[0] = ext[0];
                                                                                                lower[1] = ext[1];
												lower[2] = ext[2];
												upper[0] = ext[3];
												upper[1] = ext[4];
												upper[2] = ext[5];}

  inline AABB ( const float low[3], const float high[3] ) { lower = Vec3fa(low); upper = Vec3fa(high); }

  inline AABB ( const Vec3fa &l, const Vec3fa &u ) { lower = l; upper = u; }

  inline AABB ( const Vec3f &l, const Vec3f &u ) { lower = Vec3fa(l.x,l.y,l.z); upper = Vec3fa(u.x,u.y,u.z); }

  inline AABB& operator=( const AABB& other) { lower = other.lower; upper = other.upper; return *this; }

  inline AABB ( const AABB& other) { lower = other.lower; upper = other.upper; }

  inline void update ( const float x_val, const float y_val, const float z_val ) { lower[0] = lower[0] < x_val ? lower[0] : x_val;
                                                                                   lower[1] = lower[1] < y_val ? lower[1] : y_val;
										   lower[2] = lower[2] < z_val ? lower[2] : z_val;
										   upper[0] = upper[0] > x_val ? upper[0] : x_val;
                                                                                   upper[1] = upper[1] > y_val ? upper[1] : y_val;
										   upper[2] = upper[2] > z_val ? upper[2] : z_val; }
  
  inline void update ( const float xyz[3] )                                      { lower[0] = std::min(lower[0], xyz[0]);
                                                                                   lower[1] = std::min(lower[1], xyz[1]);
										   lower[2] = std::min(lower[2], xyz[2]);
										   upper[0] = std::max(upper[0], xyz[0]);
										   upper[1] = std::max(upper[1], xyz[1]);
										   upper[2] = std::max(upper[2], xyz[2]); }

  inline const AABB& extend(const AABB& other) { lower = min(lower, other.lower); upper = max(upper, other.upper); return *this; }

  inline Vec3fa size() const { return upper - lower; };

  inline Vec3fa center() const { return 0.5f*(lower + upper); }

  inline Vec3fa center2() const { return lower+upper; }

};

inline bool inside ( const AABB &b, const Vec3fa& p ) { return all(ge_mask(p,b.lower)) && all(le_mask(p,b.upper)); }  

inline AABB merge ( const AABB &a, const AABB &b ) { return AABB(min(a.lower,b.lower), max(a.upper,b.upper)); }

inline float volume ( const AABB &box ) { return reduce_mul(box.size()); }

inline float halfArea ( const AABB &box ) { return halfArea(box.size()); }

inline float area ( const AABB &box ) { return 2.0f*halfArea(box); }

inline bool operator ==(const AABB &a, const AABB& b) { return a.lower == b.lower &&
							a.upper == b.upper;
                                                      }

inline bool intersectBox(const AABB &b, const TravRay &ray, float& nearest_hit) {

  std::cout << "Box: " << b.lower << " " <<  b.upper << std::endl;

  float xnear, xfar, ynear, yfar, znear, zfar;
  
  if (ray.dir.x >= 0) {
    xnear = b.lower.x;
    xfar = b.upper.x;
  }
  else {
    xnear = b.upper.x;
    xfar = b.lower.x;
  }

  if (ray.dir.y >= 0) {
    ynear = b.lower.y;
    yfar = b.upper.y;
  }
  else {
    ynear = b.upper.y;
    yfar = b.lower.y;
  }    

  if (ray.dir.z >= 0) {
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

  if (tmax >= tmin) {
    nearest_hit = tmin > 0 ? tmin : 0;
    return true;
  }
  else {
    nearest_hit = inf;
    return false;
  }
    
};

inline std::ostream& operator <<( std::ostream& os, const AABB &b ) {
  return os << "Lower Corner " << b.lower << std::endl
	    << "Upper Corner " << b.upper << std::endl;
}
