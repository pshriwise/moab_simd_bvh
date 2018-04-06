

#include "testutil.hpp"
#include "Node.h"


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

int main() {

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
  
  // create a ray
  Vec3fa org(10.0, 0.0, 0.0);
  Vec3fa dir(-1.0, 0.0, 0.0);
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

  
  
  
  
  return 0;
}
