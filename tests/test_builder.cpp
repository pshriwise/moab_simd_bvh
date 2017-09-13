

#include "Node.h"
#include "Vec3.h"
#include "Traverser.h"
#include "Intersector.h"
#include "Ray.h"
#include "Vec3fa.h"
#include "Builder.h"
#include "Intersector.h"

#include <vector>

#include "testutil.hpp"

#define MAX_LEAF_SIZE 8

void test_single_primitive();
void test_random_primitives(int numPrimitives);

void* createLeaf (const BuildPrimitive *primitives, size_t numPrimitives) {
  //  assert(numPrimitives < MAX_LEAF_SIZE); needs to be re-added later
  std::vector<BuildPrimitive> p;
  p.assign(primitives, primitives+numPrimitives);

  NodeRef* leaf = encodeLeaf(&p.front(), numPrimitives); 
  return (void*)leaf;
}


int main(int argc, char** argv) {

  std::cout << "Single Primitive Test" << std::endl;
  test_single_primitive();
  std::cout << "Random Primitives Test" << std::endl;
  test_random_primitives(1E7);
  return 0;
  
}

void test_single_primitive() {
  
  BuildPrimitive p;

  p.lower_x = 0.0; p.upper_x = 4.0;
  p.lower_y = 0.0; p.upper_y = 4.0;
  p.lower_z = 0.0; p.upper_z = 4.0;

  BVHBuilder bvh(createLeaf);

  BuildSettings settings;

  NodeRef* root = bvh.Build(settings,BuildRecord(0,&p,1));
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
  
  BVHBuilder bvh(createLeaf);

  BuildSettings settings;

  BuildRecord br(0, primitives);
  
  NodeRef* root = bvh.Build(settings,br);
  
  // create a ray for intersection with the hierarchy
  Vec3fa org(10.0, 2.5, 2.5), dir(-1.0, 0.0, 0.0);
  Ray r(org, dir);
  std::cout << r << std::endl;
  
  // use the root reference node to traverse the ray
  BVHIntersector BVH;
  BVH.intersectRay(*root, r);

  bvh.stats();
  
  std::cout << r << std::endl;
}
