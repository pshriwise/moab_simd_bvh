

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
void test_hollow_box();

void build_hollow_cube(const float& x_min, const float& x_width, const size_t& x_prims,
		       const float& y_min, const float& y_width, const size_t& y_prims,
		       const float& z_min, const float& z_width, const size_t& z_prims,
		       std::vector<BuildPrimitive>& primitives) {

  std::vector<BuildPrimitive> prims;
  
  float x_step = x_width/(float)x_prims;
  float y_step = y_width/(float)y_prims;
  float z_step = z_width/(float)z_prims;

  int prim_id = 0;
  
  for(size_t i = 0; i <= x_prims; i++) {
    for(size_t j = 0; j <= y_prims; j++) {
      for(size_t k = 0; k <= z_prims;) {

	if ( !((i == 0 || i == x_prims || j == 0 || j == y_prims)) ) {	  
	  if (k == 1) {
	    k = z_prims; continue;
	  }
	}

        BuildPrimitive p = BuildPrimitive(x_min+i*x_step,
					  y_min+j*y_step,
					  z_min+k*y_step,
					  0,
					  x_min+(i+1)*x_step,
					  y_min+(j+1)*y_step,
					  z_min+(k+1)*z_step,
					  prim_id++);

	prims.push_back(p);

	k++;
	
      }
    }
  }

  primitives = prims;


  return;
}




void* createLeaf (BuildPrimitive *primitives, size_t numPrimitives) {
  //  assert(numPrimitives < MAX_LEAF_SIZE); needs to be re-added later
  NodeRef* leaf = encodeLeaf(primitives, numPrimitives); 
  return (void*)leaf;
}



int main(int argc, char** argv) {

  std::cout << "Single Primitive Test" << std::endl << std::endl; 
  test_single_primitive();
  std::cout << "Hollow Box Primitives Test" << std::endl << std::endl;
  test_hollow_box();
  std::cout << "Random Primitives Test" << std::endl << std::endl;
  // test_random_primitives(1E3);
  return 0;
  
}

void test_single_primitive() {
  
  BuildPrimitive p;

  p.lower_x = 0.0; p.upper_x = 4.0;
  p.lower_y = 0.0; p.upper_y = 4.0;
  p.lower_z = 0.0; p.upper_z = 4.0;

  BVHBuilder bvh(createLeaf);

  BuildSettings settings;

  BuildRecord br(0, &p, 1);
  
  NodeRef* root = bvh.Build(settings,br);
}

void test_random_primitives(int numPrimitives) {

  std::vector<BuildPrimitive> primitives;
  int primID = 0;
  for(int i = 0; i < numPrimitives; i++){  
    BuildPrimitive p;

    p.lower_x = float(drand48()); p.upper_x = float(drand48());
    p.lower_y = float(drand48()); p.upper_y = float(drand48());
    p.lower_z = float(drand48()); p.upper_z = float(drand48());
    p.primID = primID++;
    p.sceneID = 0;
    
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


void test_hollow_box() {

  std::vector<BuildPrimitive> primitives;
  
  build_hollow_cube(0.0, 10.0, 20,
		    0.0, 10.0, 20,
		    0.0, 10.0, 20,
		    primitives);

  BVHBuilder bvh(createLeaf);

  BuildSettings settings;

  BuildRecord br(0, primitives);
  
  NodeRef* root = bvh.Build(settings,br);

  bvh.stats();

    // create a ray for intersection with the hierarchy
  Vec3fa org(5,5,5), dir(-1.0, 0.0, 0.0);
  Ray r(org, dir);
  
  // use the root reference node to traverse the ray
  BVHIntersector BVH;
  BVH.intersectRay(*root, r);

  std::cout << r << std::endl;

  
}
  
