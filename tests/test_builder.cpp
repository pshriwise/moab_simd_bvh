

#include "Node.h"
#include "Vec3.h"
#include "Traverser.h"
#include "Intersector.h"
#include "Ray.h"
#include "Vec3fa.h"
#include "Builder.h"
#include "Intersector.h"

#include <vector>
#include <math.h>

#include "testutil.hpp"

#define MAX_LEAF_SIZE 8

void test_single_primitive();
void test_random_primitives(int numPrimitives);
void test_hollow_box(float x_min, float y_min, float z_min,
		     float x_max, float y_max, float z_max,
		     int num_x,   int num_y,   int num_z);
void test_hollow_box_big();
void check_leaf_pointers(NodeRef *root);

void check_leaf_pointers(NodeRef *root,
			 std::vector<BuildPrimitive*>& leaf_pointers,
			 std::vector<BuildPrimitive*>& duplicates);

void build_hollow_box(const float& x_min, const float& x_width, const size_t& x_prims,
		      const float& y_min, const float& y_width, const size_t& y_prims,
		      const float& z_min, const float& z_width, const size_t& z_prims,
		      std::vector<BuildPrimitive>& primitives);

int main(int argc, char** argv) {

  std::cout << "Single Primitive Test" << std::endl << std::endl; 
  test_single_primitive();
  std::cout << "Random Primitives Test" << std::endl << std::endl;
  test_random_primitives(1E3);

  std::cout << "Hollow Box Test" << std::endl << std::endl;
  test_hollow_box(0.0, 0.0, 0.0,
		  20.0, 20.0, 20.0,
		  4, 4, 4);
  std::cout << "Big Hollow Box Test" << std::endl << std::endl;
  test_hollow_box_big();
  std::cout << "Bigger Hollow Box Test" << std::endl << std::endl;
  //  test_hollow_box_bigger();
  
  
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

  bvh.stats();
  
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

  bvh.stats();

  // create a ray for intersection with the hierarchy
  Vec3fa org(10.0, 2.5, 2.5), dir(-1.0, 0.0, 0.0);
  Ray r(org, dir);
  
  // use the root reference node to traverse the ray
  BVHIntersector BVH;
  BVH.intersectRay(*root, r);

  return;
}


void test_hollow_box(float x_min, float y_min, float z_min,
		     float x_max, float y_max, float z_max,
		     int num_x,   int num_y,   int num_z) {

  std::vector<BuildPrimitive> primitives;

  Vec3f min(x_min, y_min, z_min);

  Vec3f max(x_max, y_max, z_max);

  build_hollow_box( x_min, x_max, num_x,
		    y_min, y_max, num_y,
		    z_min, z_max, num_z,
		    primitives);

  BVHBuilder bvh(create_leaf);

  BuildSettings settings;

  BuildState br(0, primitives);
  
  NodeRef* root = bvh.Build(settings,br);

  bvh.stats();

  check_leaf_pointers(root);

  // some scoping from the inside of the box
  
  // create a ray origin for intersection with the hierarchy,
  // shifted in each dimension by 0.1 units
  Vec3fa org(0.1+ (min.x+max.x)/2.0,
	     0.1+ (min.y+max.y)/2.0,
	     0.1+ (min.z+max.z)/2.0);

  // use the root reference node to traverse the ray
  BVHIntersector BVH;
  Ray r;
  Vec3fa dir;
  float dist;
  // fire a ray in the positive and negative direction for each ray
  for( int dim = 0; dim < 3 ; dim++ ) {

    // negative direction
    dir = Vec3fa(0.0);
    dir[dim] = -1.0;
    
    r = Ray(org, dir);

    BVH.intersectRay(*root, r);

    // distance to intersection should be equal to the distance from the minimum
    // box edge to the ray origin
    dist = org[dim]-min[dim];
    // minus the size of the build primitive
    dist -=(min[dim]+max[dim])/num_x;

    std::cout << dim << std::endl;
    std::cout << r << std::endl;
    CHECK_REAL_EQUAL(dist, r.tfar, 1e-06);

    // now do the same for the positive direction
    dir = Vec3fa(0.0);
    dir[dim] = 1.0;
    
    r = Ray(org, dir);

    BVH.intersectRay(*root, r);

    // distance to intersection should be equal to the distance from the maximum
    // box edge to the ray origin
    dist = max[dim]-org[dim];
    // minus the size of the build primitive
    dist -=(min[dim]+max[dim])/num_x;

    CHECK_REAL_EQUAL(dist, r.tfar, 1e-06);
  }

  // create a ray origin at the exact center of the box
  org = Vec3fa((min.x+max.x)/2.0,
	       (min.y+max.y)/2.0,
	       (min.z+max.z)/2.0);

  
  float corner_dist = 3.0f * pow((max.x-org.x)-(max.x-min.x)/num_x, 2.0f);
  corner_dist = sqrt(corner_dist);
    

  Vec3fa corner_dirs[8] = { Vec3fa(  1,  1,   1),
			    Vec3fa( -1,  1,   1),
			    Vec3fa( -1, -1,   1),
			    Vec3fa( -1, -1,  -1),
			    Vec3fa(  1, -1,  -1),
			    Vec3fa(  1,  1,  -1),
			    Vec3fa( -1,  1,  -1),
			    Vec3fa(  1, -1,  1)};

  // fire a ray into each corner
  for(int i = 0 ; i < 8 ; i++) {
    dir = corner_dirs[i];
    
    r = Ray(org,dir);

    BVH.intersectRay(*root,r);
      
    CHECK_REAL_EQUAL(corner_dist, r.tfar, 1e-06);
      
  }

  // some cleanup
  delete root;
 
}


void test_hollow_box_big() {

  test_hollow_box(0.0, 0.0, 0.0,
		  20.0, 20.0, 20.0,
		  10, 10, 10);
  
}

void check_leaf_pointers(NodeRef *root) {
  std::vector<BuildPrimitive*> bps;
  std::vector<BuildPrimitive*> duplicate_ptrs;
  
  check_leaf_pointers(root, bps, duplicate_ptrs);

  if ( duplicate_ptrs.size() > 0 ) {
    std::cout << "DUPLICATE LEAF PONTERS" << std::endl;
    for(unsigned int i = 0; i < duplicate_ptrs.size(); i++ ) {
      std::cout << "Pointer Value: " << duplicate_ptrs[i] << std::endl;
      std::cout << *(duplicate_ptrs[i]) << std::endl;
    }
  }
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

void build_hollow_box(const float& x_min, const float& x_max, const size_t& x_prims,
		       const float& y_min, const float& y_max, const size_t& y_prims,
		       const float& z_min, const float& z_max, const size_t& z_prims,
		       std::vector<BuildPrimitive>& primitives) {

  std::vector<BuildPrimitive> prims;

  float x_width = x_max - x_min;
  float y_width = y_max - y_min;
  float z_width = z_max - z_min;
  
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
