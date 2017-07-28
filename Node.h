#pragma once

#include "Vec3.h"
#include "constants.h"

#define N 4

static const size_t emptyNode = 8;

struct AANode
{
 
  //  inline AANode& child(size_t i) { assert(i<N); return children[i]; }
  //inline const AANode& child(size_t i) const { assert(i<N); return children[i]; }

  // empty constructor
  inline AANode() {}


  inline void clear() { lower_x = lower_y = lower_z = neg_inf;
    upper_x = upper_y = upper_z = inf; }
			//			for(size_t i=0; i < N ; i++) children[i] = emptyNode; }

  inline void setBounds(const AABB& bounds) { lower_x = bounds.lower.x; lower_y = bounds.lower.y; lower_z = bounds.lower.z;
                                              upper_x = bounds.upper.x; upper_y = bounds.upper.y; upper_z = bounds.upper.z; }
  
  inline AABB bounds() const { const Vec3f lower(lower_x, lower_y, lower_z);
                               const Vec3f upper(upper_x, upper_y, upper_z);
			       return AABB(lower, upper); }
  
  float lower_x, upper_x, lower_y, upper_y, lower_z, upper_z;

  //  AANode children[N]; 
  
};

  
inline size_t intersectBox(const AANode &node, const TravRay &ray, const float &tnear, const float &tfar, float &dist) {
  const float tNearX = (*(float*)((const char*)&node.lower_x + ray.nearX)- ray.org.x) * ray.rdir.x;
  const float tNearY = (*(float*)((const char*)&node.lower_x + ray.nearY) - ray.org.y) * ray.rdir.y;
  const float tNearZ = (*(float*)((const char*)&node.lower_x + ray.nearZ) - ray.org.z) * ray.rdir.z;
  const float tFarX = (*(float*)((const char*)&node.lower_x + ray.farX) - ray.org.x) * ray.rdir.x;
  const float tFarY = (*(float*)((const char*)&node.lower_x + ray.farY) - ray.org.y) * ray.rdir.y;
  const float tFarZ = (*(float*)((const char*)&node.lower_x + ray.farZ) - ray.org.z) * ray.rdir.z;

  const float round_down = 1.0f-2.0f*float(ulp); // FIXME: use per instruction rounding for AVX512
  const float round_up   = 1.0f+2.0f*float(ulp);

  const float tNear = std::max(std::max(tNearX, tNearY), tNearZ);
  const float tFar = std::min(std::min(tFarX, tFarY), tFarZ);
  const bool vmask = (round_down*tNear <= round_up*tFar);
  dist = tNear;
  return vmask;
};

