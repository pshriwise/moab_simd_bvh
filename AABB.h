
#include <cstring>
#include <algorithm>


#include "Vec3.h"
#include "Ray.h"
#include "constants.h"


struct AABB {

  Vec3f lower, upper;

  inline AABB() {}

  inline AABB ( float x_min, float y_min, float z_min, float x_max, float y_max, float z_max) { lower[0] = x_min;
                                                                                               lower[1] = y_min;
											       lower[2] = z_min;
											       upper[0] = x_max;
											       upper[1] = y_max;
											       upper[2] = z_max; }
  
  inline AABB ( const float ext[6] )                                                          { lower[0] = ext[0];
                                                                                                lower[1] = ext[1];
												lower[2] = ext[2];
												upper[0] = ext[3];
												upper[1] = ext[4];
												upper[2] = ext[5];}

  inline AABB ( const float low[3], const float high[3] ) { lower = Vec3f(low); upper = Vec3f(high); }

  inline AABB ( const Vec3f &l, const Vec3f &u ) { lower = l; upper = u; }
  
  inline AABB& operator=( const AABB& other) { lower = other.lower; upper = other.upper; return *this; }

  inline AABB ( const AABB& other) { lower = other.lower; upper = other.upper; }

  inline void update ( const float x_val, const float y_val, const float z_val ) { lower[0] = std::min(lower[0], x_val);
                                                                                   lower[1] = std::min(lower[1], y_val);
										   lower[2] = std::min(lower[2], z_val);
										   upper[0] = std::max(upper[0], x_val);
										   upper[1] = std::max(upper[1], y_val);
										   upper[2] = std::max(upper[2], z_val); }

  inline void update ( const float xyz[3] )                                      { lower[0] = std::min(lower[0], xyz[0]);
                                                                                   lower[1] = std::min(lower[1], xyz[1]);
										   lower[2] = std::min(lower[2], xyz[2]);
										   upper[0] = std::max(upper[0], xyz[0]);
										   upper[1] = std::max(upper[1], xyz[1]);
										   upper[2] = std::max(upper[2], xyz[2]); }

  inline const AABB& extend(const AABB& other) { lower = min(lower, other.lower); upper = max(upper, other.upper); return *this; }

  inline Vec3f size() const { return upper - lower; };

  inline Vec3f center() const { return 0.5f*(lower + upper); }

  inline Vec3f center2() const { return lower+upper; }

};

inline bool inside ( const AABB &b, const Vec3f& p ) { return all(ge_mask(p,b.lower)) && all(le_mask(p,b.upper)); }  

inline AABB merge ( const AABB &a, const AABB &b ) { return AABB(min(a.lower,b.lower), max(a.upper,b.upper)); }

inline float volume ( const AABB &box ) { return reduce_mul(box.size()); }

inline float halfArea ( const AABB &box ) { return halfArea(box.size()); }

inline float area ( const AABB &box ) { return 2.0f*halfArea(box); }


inline size_t intersectBox(const AABB &box, const TravRay &ray, const float &tnear, const float &tfar, float &dist) {
  const float tNearX = (*(float*)((const char*)&box.lower + ray.nearX));// - ray.org.x) * ray.rdir.x;
  const float tNearY = (*(float*)((const char*)&box.lower + ray.nearY));// - ray.org.y) * ray.rdir.y;
  const float tNearZ = (*(float*)((const char*)&box.lower + ray.nearZ)); // - ray.org.z) * ray.rdir.z;
  const float tFarX = (*(float*)((const char*)&box.lower + ray.farX));// - ray.org.x) * ray.rdir.x;
  const float tFarY = (*(float*)((const char*)&box.lower + ray.farY)); // - ray.org.y) * ray.rdir.y;
  const float tFarZ = (*(float*)((const char*)&box.lower + ray.farZ));// - ray.org.z) * ray.rdir.z;

  std::cout << tNearX << std::endl;
  std::cout << tNearY << std::endl;
  std::cout << tNearZ << std::endl;
  std::cout << tFarX << std::endl;
  std::cout << tFarY << std::endl;
  std::cout << tFarZ << std::endl;

  const float round_down = 1.0f-2.0f*float(ulp); // FIXME: use per instruction rounding for AVX512
  const float round_up   = 1.0f+2.0f*float(ulp);

  const float tNear = std::min(std::max(tNearX, tNearY), tNearZ);
  const float tFar = std::max(std::max(tFarX, tFarY), tFarZ);
  const bool vmask = (round_down*tNear <= round_up*tFar);
  dist = tNear;
  return vmask;
}
