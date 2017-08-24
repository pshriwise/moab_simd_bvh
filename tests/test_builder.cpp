

#include "Node.h"
#include "Vec3.h"
#include "Traverser.h"
#include "Intersector.h"
#include "Ray.h"
#include "Vec3fa.h"
#include "Builder.h"

#include <vector>

#include "testutil.hpp"


void*  createLeaf (const BuildPrimitive *primitives, size_t numPrimitives) {
  return NULL;
}


int main(int argc, char** argv) {

  BuildPrimitive p;

  p.lower_x = 0.0; p.upper_x = 4.0;
  p.lower_y = 0.0; p.upper_y = 4.0;
  p.lower_z = 0.0; p.upper_z = 4.0;

  BVHBuilder bvh;

  BuildSettings settings;

  NodeRef* root = bvh.Build(settings,&p,1,createLeaf);

  return 0;
  

}
