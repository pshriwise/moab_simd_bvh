
#include <cstring>
#include <algorithm>

#include "Vec3.h"

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

  inline AABB ( const float low[3], float high[3] ) { lower = Vec3f(low); upper = Vec3f(high); }
  
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


};

  inline bool inside ( const AABB &b, const Vec3f& p ) { return all(ge_mask(p,b.lower)) && all(le_mask(p,b.upper)); }  
