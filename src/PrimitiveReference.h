
#pragma once

#include "Vec3fa.h"
#include "AABB.h"

struct PrimRef{
  
  inline PrimRef () {}

  inline PrimRef( const Vec3fa& lower, const Vec3fa& upper, void* p, int i) : lower(lower), upper(upper), primitivePtr(p) { this->upper.a = i; }
  
  inline PrimRef (const AABB& bounds, unsigned int geomID, unsigned int primID)
  {
    lower = bounds.lower; upper.a = geomID;
    upper = bounds.upper; upper.a = primID;
  }
  
  inline const Vec3fa center() const {
    return (lower+upper)/2.0f;
  }

  inline const Vec3fa center2() const {
    return lower+upper;
  }

  inline const AABB bounds() const {
    return AABB(lower,upper);
  }

  inline unsigned size() const {
    return 1;
  }

  inline unsigned geomID() const {
    return lower.a;
  }

  inline unsigned primID() const {
    return upper.a;
  }

public:
  Vec3fa lower, upper;
  void* primitivePtr;
};
