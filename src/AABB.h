
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

