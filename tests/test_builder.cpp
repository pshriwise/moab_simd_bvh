

#include "Node.h"
#include "Vec3.h"
#include "Traverser.h"
#include "Intersector.h"
#include "Ray.h"
#include "Vec3fa.h"
#include "Builder.h"

#include <vector>

#include "testutil.hpp"

void test_single_primitive();
void test_random_primitives(int numPrimitives);

void*  createLeaf (const BuildPrimitive *primitives, size_t numPrimitives) {
  NodeRef* leaf = new NodeRef(tyLeaf);
  return (void*)leaf;
}


int main(int argc, char** argv) {

  test_single_primitive();
  test_random_primitives(20);
  return 0;
  

}

void test_single_primitive() {
  
  BuildPrimitive p;

  p.lower_x = 0.0; p.upper_x = 4.0;
  p.lower_y = 0.0; p.upper_y = 4.0;
  p.lower_z = 0.0; p.upper_z = 4.0;

  BVHBuilder bvh;

  BuildSettings settings;

  NodeRef* root = bvh.Build(settings,&p,1,createLeaf);
}

void test_random_primitives(int numPrimitives) {

  std::vector<BuildPrimitive> primitives;

  for(int i = 0; i < numPrimitives; i++){  
    BuildPrimitive p;

    p.lower_x = float(drand48()); p.upper_x = float(drand48());
    p.lower_y = float(drand48()); p.upper_y = float(drand48());
    p.lower_z = float(drand48()); p.upper_z = float(drand48());

    primitives.push_back(p);
  }
  
  BVHBuilder bvh;

  BuildSettings settings;

  NodeRef* root = bvh.Build(settings,&(primitives[0]),(size_t)primitives.size(),createLeaf);
}
