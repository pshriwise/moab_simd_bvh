
#pragma once

#include "Ray.h"
#include "Node.h"
#include "Intersector.h"
#include "Stack.h"
#include "Primitive.h"

#include <vector>

// #define VERBOSE_MODE

#ifdef VERBOSE_MODE
  #include <bitset>
#endif

template <typename T, typename V, typename P, typename I>
class BVHTraverserT {

    static const size_t stackSize = 1+(N-1)*BVH_MAX_DEPTH;

 public:
  //  void intersectRay(NodeRef root, Ray& ray);
  
  static inline bool intersect(NodeRef& node, const TravRay& ray, const vfloat4& tnear, const vfloat4& tfar, vfloat4& dist, size_t& mask);

  inline void intersectRay (NodeRef root, RayT<V,P,I> & ray);

};

