

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
void check_leaf_pointers(NodeRef *root, std::vector<BuildPrimitive*>& leaf_pointers, std::vector<BuildPrimitive*>& Duplicates);

void build_hollow_cube(const float& x_min, const float& x_width, const size_t& x_prims,
		       const float& y_min, const float& y_width, const size_t& y_prims,
		       const float& z_min, const float& z_width, const size_t& z_prims,
		       std::vector<BuildPrimitive>& primitives);

int main(int argc, char** argv) {

  std::cout << "Single Primitive Test" << std::endl << std::endl; 
  test_single_primitive();
  std::cout << "Hollow Box Primitives Test" << std::endl << std::endl;
  test_hollow_box();
  //  std::cout << "Random Primitives Test" << std::endl << std::endl;
  // test_random_primitives(1E3);
  return 0;
  
}

void test_single_primitive() {
  
  BuildPrimitive p;

  p.lower_x = 0.0; p.upper_x = 4.0;
  p.lower_y = 0.0; p.upper_y = 4.0;
  p.lower_z = 0.0; p.upper_z = 4.0;
  p.sceneID = 0;   p.primID = 1;
  
  BVHBuilder bvh(create_leaf);

  BuildSettings settings;

  BuildState br(0, &p, 1);
  
  NodeRef* root = bvh.Build(settings,br);

  BVHIntersector INT;

  // create a ray for intersection with the hierarchy
  Vec3fa org(5.0,2.0,2.0), dir(-1.0, 0.0, 0.0);
  Ray r(org, dir);
  
  // use the root reference node to traverse the ray
  INT.intersectRay(*root, r);

  CHECK_REAL_EQUAL(1.0f, r.tfar, 1e-06);

  org = Vec3fa(2.0,2.0,2.0);

  r = Ray(org, dir);
  
  // use the root reference node to traverse the ray
  INT.intersectRay(*root, r);
  
  CHECK_REAL_EQUAL(0.0f, r.tfar, 1e-06);
  
  delete root;
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
  
  BVHBuilder bvh(create_leaf);

  BuildSettings settings;

  BuildState br(0, primitives);
  
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
  
  build_hollow_cube(0.0, 20.0, 4,
		    0.0, 20.0, 4,
		    0.0, 20.0, 4,
		    primitives);

  BVHBuilder bvh(create_leaf);

  BuildSettings settings;

  BuildState br(0, primitives);
  
  NodeRef* root = bvh.Build(settings,br);

  bvh.stats();

  std::vector<BuildPrimitive*> v,d;
  check_leaf_pointers(root, v, d);

  std::cout << "DUPLICATE LEAF PONTERS" << std::endl;
  for(unsigned int i = 0; i < d.size(); i++ ) {
    std::cout << "Pointer Value: " << d[i] << std::endl;
    std::cout << *(d[i]) << std::endl;
  }

  // create a ray for intersection with the hierarchy
  Vec3fa org(10.1,10.1,10.1), dir(-1.0, 0.0, 0.0);
  Ray r(org, dir);
  
  // use the root reference node to traverse the ray
  BVHIntersector BVH;
  BVH.intersectRay(*root, r);

  std::cout << r << std::endl;

  CHECK_REAL_EQUAL(5.1f, r.tfar, 1e-06);

  dir = Vec3fa(1.0, 0.0, 0.0);
  r = Ray(org,dir);

  std::cout << r << std::endl;

  BVH.intersectRay(*root, r);

  std::cout << r << std::endl;
  
  CHECK_REAL_EQUAL(4.9f, r.tfar, 1e-06);

  delete root;
  
}

void check_leaf_pointers(NodeRef *root, std::vector<BuildPrimitive*>& leaf_pointers, std::vector<BuildPrimitive*>& duplicates) {

  NodeRef current = *root;

  // if the node is empty, move on
  if( current.isEmpty() ) {
    return;
  }
  
  // if this is a leaf, make sure it's pointer doesn't already exist in the set of leaves
  if( current.isLeaf() ){

    // get the leaf info
    size_t num;
    BuildPrimitive* p = (BuildPrimitive*)current.leaf(num);
    
    
    if ( std::find(leaf_pointers.begin(), leaf_pointers.end(), p) != leaf_pointers.end() ) {
    //   std::cout << "Duplicate nodes exist in the tree for some reason." << std::endl;
    //   std::cout << "Pointer is: " << std::endl;
    //   std::cout << p << std::endl;
    //   std::cout << "Current primitive is: " << std::endl;
    //   std::cout << *p << std::endl;
      duplicates.push_back(p);
    }
   
    leaf_pointers.push_back(p);

  }
  // if this is not a leaf, check all children
  else{
    for( size_t i = 0; i < N ; i++ ) {
      NodeRef* child = &current.bnode()->child(i);
      check_leaf_pointers(child, leaf_pointers, duplicates);
    }
  }
  
  return;  
}

void build_hollow_cube(const float& x_min, const float& x_width, const size_t& x_prims,
		       const float& y_min, const float& y_width, const size_t& y_prims,
		       const float& z_min, const float& z_width, const size_t& z_prims,
		       std::vector<BuildPrimitive>& primitives) {

  std::vector<BuildPrimitive> prims;
  
  float x_step = x_width/(float)x_prims;
  float y_step = y_width/(float)y_prims;
  float z_step = z_width/(float)z_prims;

  int prim_id = 0;
  
  for(size_t i = 0; i < x_prims; i++) {
    for(size_t j = 0; j < y_prims; j++) {
      for(size_t k = 0; k < z_prims;) {

	if ( !((i == 0 || i == x_prims-1 || j == 0 || j == y_prims-1)) ) {	  
	  if (k == 1) {
	    k = z_prims-1; continue;
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
