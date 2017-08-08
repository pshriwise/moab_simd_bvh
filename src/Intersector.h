


#pragma once

#include "Ray.h"
#include "Node.h"


class BVHIntersector {
  
  static const size_t stackSize = 1+(N-1)*BVH_MAX_DEPTH;

 public:
  void intersectRay(NodeRef root, Ray& ray);
  
  static inline bool intersect(NodeRef& node, const TravRay& ray, const vfloat4& tnear, const vfloat4& tfar, vfloat4& dist, size_t& mask) {
    if(node.isLeaf()) return false;
    mask = intersectBox(*node.node(),ray,tnear,tfar,dist);
    return true;
  }



};
