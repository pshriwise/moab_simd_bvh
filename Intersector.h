


#pragma once

#include "Ray.h"
#include "Node.h"


class BVHIntersector {
  
  static const size_t stackSize = 1+(N-1)*BVH_MAX_DEPTH;

 public:
  static void intersect(NodeRef root, Ray& ray);

};
