

#include "testutil.hpp"
#include "Node.h"

#define EPS 1e-04

// returns an OBB constructed using the bounding points of an AABB
OBB obb_from_aabb(const AABB& aabb) {

  Vec3fa lo = aabb.lower;
  Vec3fa hi = aabb.upper;
  // generate a controlled OBB 
  std::vector<float> xs, ys, zs;
  xs.push_back(lo.x); ys.push_back(lo.y); zs.push_back(lo.z);
  xs.push_back(hi.x); ys.push_back(lo.y); zs.push_back(lo.z);
  xs.push_back(hi.x); ys.push_back(hi.y); zs.push_back(lo.z);
  xs.push_back(lo.x); ys.push_back(hi.y); zs.push_back(lo.z);
  xs.push_back(lo.x); ys.push_back(lo.y); zs.push_back(hi.z);
  xs.push_back(hi.x); ys.push_back(lo.y); zs.push_back(hi.z);
  xs.push_back(hi.x); ys.push_back(hi.y); zs.push_back(hi.z);
  xs.push_back(lo.x); ys.push_back(hi.y); zs.push_back(hi.z);
  
  OBB box(&xs.front(), &ys.front(), &zs.front(), xs.size());

  return box;
}

void uniform_node_tests();
void nonuniform_node_tests();
void unaligned_node_tests();

int main() {
 
  uniform_node_tests();
  nonuniform_node_tests();
  unaligned_node_tests();
  
  return 0;
}

void uniform_node_tests() {
  
  AABB aabb(Vec3fa(0.0), Vec3fa(5.0));
  OBB box = obb_from_aabb(aabb);

  // setup a node with that box
  UANode node = UANode();
  node.setBounds(box);

  // setup some intersection information
  vfloat4 z(zero), i(inf), ni(neg_inf);
  vfloat4 dist(100.0);
  size_t result;

  vfloat4 expected_dist;
  size_t expected_result;

  for(size_t dim = 0 ; dim < 3; dim++) {
    // create a ray
    Vec3fa org(0.0);
    Vec3fa dir(0.0);

    org[dim] = 10.0;
    dir[dim] = -1.0;
    
    TravRay r(org, dir);

    // intersect the node
    result = intersectBox(node, r, z, i, dist);

    // check the results
    expected_dist = 5.0;
    expected_result = 15;
    CHECK_EQUAL(expected_result, result);
    CHECK_VFLOATREAL_EQUAL(expected_dist, dist);

    // intersect the node
    result = intersectBox(node, r, z, z, dist);

    // check the results
    expected_result = 0;
    CHECK_EQUAL(expected_result, result);

    // setup a new ray (reversed direction)
    r.dir = -r.dir;

    // intersect the node
    result = intersectBox(node, r, z, i, dist);

    // check the results
    expected_result = 0;
    CHECK_EQUAL(expected_result, result);
  }
  
  return;
}
void nonuniform_node_tests() {
  
  AABB aabb0(Vec3fa(0.0), Vec3fa(5.0));
  OBB box0 = obb_from_aabb(aabb0);
  AABB aabb1(Vec3fa(0.0), Vec3fa(4.0));
  OBB box1 = obb_from_aabb(aabb1);
  AABB aabb2(Vec3fa(0.0), Vec3fa(3.0));
  OBB box2 = obb_from_aabb(aabb2);
  AABB aabb3(Vec3fa(0.0), Vec3fa(2.0));
  OBB box3 = obb_from_aabb(aabb3);

  // setup a node with that box
  UANode node = UANode();
  node.setBound(0, box0);
  node.setBound(1, box1);
  node.setBound(2, box2);
  node.setBound(3, box3);

  // setup some intersection information
  vfloat4 z(zero), i(inf), ni(neg_inf);
  vfloat4 dist(100.0);
  size_t result;

  vfloat4 expected_dist;
  size_t expected_result;

  for(size_t dim = 0; dim < 3; dim++) { 
    // create a ray
    Vec3fa org(0.0);
    Vec3fa dir(0.0);

    org[dim] = 10.0;
    dir[dim] = -1.0;
    
    TravRay r(org, dir);

    // intersect the node
    result = intersectBox(node, r, z, i, dist);

    // check the results
    expected_dist = vfloat4(5.0, 6.0, 7.0, 8.0);;
    expected_result = 15;
    CHECK_EQUAL(expected_result, result);
    CHECK_VFLOATREAL_EQUAL(expected_dist, dist);

    // intersect the node
    result = intersectBox(node, r, z, z, dist);

    // check the results
    expected_result = 0;
    CHECK_EQUAL(expected_result, result);

    // set a non-uniform ray length
    vfloat4 tfar(10.0, 10.0, 10.0, 0.0);

    // intersect the node
    result = intersectBox(node, r, z, tfar, dist);

    // check the results
    expected_result = 7;
    CHECK_EQUAL(expected_result, result);

    // set a non-uniform ray length
    tfar = vfloat4(10.0, 10.0, 0.0, 0.0);

    // intersect the node
    result = intersectBox(node, r, z, tfar, dist);

    // check the results
    expected_result = 3;
    CHECK_EQUAL(expected_result, result);

    // set a non-uniform ray length
    tfar = vfloat4(5.5, 5.5, 5.5, 5.5);

    // intersect the node
    result = intersectBox(node, r, z, tfar, dist);

    // check the results
    expected_result = 1;
    CHECK_EQUAL(expected_result, result);

    // set a non-uniform ray length
    tfar = vfloat4(7.5, 7.5, 7.5, 7.5);

    // intersect the node
    result = intersectBox(node, r, z, tfar, dist);

    // check the results
    expected_result = 7;
    CHECK_EQUAL(expected_result, result);

    // set a non-uniform ray length
    tfar = vfloat4(8.5, 8.5, 8.5, 8.5);

    // intersect the node
    result = intersectBox(node, r, z, tfar, dist);

    // check the results
    expected_result = 15;
    CHECK_EQUAL(expected_result, result);

    // setup a new ray (reversed direction)
    r.dir = -r.dir;

    // intersect the node
    result = intersectBox(node, r, z, i, dist);

    // check the results
    expected_result = 0;
    CHECK_EQUAL(expected_result, result);    
  }

  return;
}

void unaligned_node_tests() {

  std::vector<float> xs, ys, zs;

  xs.push_back(10.0); ys.push_back( 5.0); zs.push_back( 0.0);
  xs.push_back(-5.0); ys.push_back( 0.0); zs.push_back( 0.0);
  xs.push_back( 5.0); ys.push_back(10.0); zs.push_back( 0.0);
  xs.push_back( 0.0); ys.push_back(-5.0); zs.push_back( 0.0);
  
  xs.push_back(10.0); ys.push_back( 5.0); zs.push_back(10.0);
  xs.push_back(-5.0); ys.push_back( 0.0); zs.push_back(10.0);
  xs.push_back( 5.0); ys.push_back(10.0); zs.push_back(10.0);
  xs.push_back( 0.0); ys.push_back(-5.0); zs.push_back(10.0);

  OBB box(&xs.front(), &ys.front(), &zs.front(), xs.size());
  
  // setup some intersection information
  vfloat4 z(zero), i(inf), ni(neg_inf);
  vfloat4 dist(100.0);
  size_t result;

  vfloat4 expected_dist;
  size_t expected_result;

  UANode node = UANode();
  node.setBounds(box);
  
  size_t dim = 0;
  
  // create a ray
  Vec3fa org(20.0, 0.0, 0.0);
  Vec3fa dir(-1.0,  0.0,  0.0);

  TravRay r(org,dir);
  
  // intersect the node
  result = intersectBox(node, r, z, i, dist);
  
  // check the results
  expected_dist = vfloat4(15.0);
  expected_result = 15;
  CHECK_EQUAL(expected_result, result);
  CHECK_VFLOATREAL_EQUAL(expected_dist, dist);

  r.org = Vec3fa(20.0, 5.0, 0.0);

  // intersect the node
  result = intersectBox(node, r, z, i, dist);
  
  // check the results
  expected_dist = vfloat4(10.0);
  expected_result = 15;
  CHECK_EQUAL(expected_result, result);
  CHECK_VFLOATREAL_EQUAL(expected_dist, dist);

  r.org = Vec3fa(20.0, 7.5, 0.0);

  // intersect the node
  result = intersectBox(node, r, z, i, dist);
  
  // check the results
  expected_dist = vfloat4(12.5);
  expected_result = 15;
  CHECK_EQUAL(expected_result, result);
  CHECK_VFLOATREAL_EQUAL(expected_dist, dist);

  r.org = Vec3fa(20.0, -2.5, 0.0);

  // intersect the node
  result = intersectBox(node, r, z, i, dist);
  
  // check the results
  expected_dist = vfloat4(17.5);
  expected_result = 15;
  CHECK_EQUAL(expected_result, result);
  CHECK_VFLOATREAL_EQUAL(expected_dist, dist);

  OBB bounds = node.bounds();

  CHECK_REAL_EQUAL(area(box), area(bounds), EPS);
  
  return;
}
