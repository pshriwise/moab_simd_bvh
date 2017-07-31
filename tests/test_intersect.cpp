

#include "AABB.h"
#include "Ray.h"
#include "testutil.hpp"
#include "Node.h"
#include "vfloat.h"

void test_intersect();

int main (int argc, char** argv) {

  test_intersect();

  return 0;
}


void test_intersect() {

  AABB box(0.0, 0.0, 0.0,
	   5.0, 5.0, 5.0);

  AANode n = AANode();

  n.setBounds(box);
  
  Vec3f org = Vec3f( 9.0, 3.0, 3.0);
  Vec3f dir = Vec3f(-1.0, 0.0, 0.0);

  TravRay r(org, dir);

  CHECK(!inside(box,org));
  
  vfloat4 dist(100.0);
  bool result;

  // setup values for testing rays
  vfloat4 z(zero), i(inf), ni(neg_inf);
  vfloat4 expected_dist(4.0);

  // test ray incident on box, infinite length
  // (intersection)
  result = intersectBox(n, r, z, i, dist);
  CHECK_VFLOATREAL_EQUAL(expected_dist, dist);
  CHECK(result);

  // test ray incident on box, zero length
  // (no intersection)
  result = intersectBox(n, r, z, z, dist);
  CHECK(!result);

  // test ray going away from box, infinite length
  // (no intersection)
  r = TravRay(org,-dir);
  result = intersectBox(n, r, z, i, dist);
  CHECK(!result);

  // test ray going away from box, infinite positive
  // and negative lengths
  // (intersection)
  expected_dist = vfloat4(-9.0);
  r = TravRay(org,-dir);
  result = intersectBox(n, r, ni, i, dist);
  CHECK_VFLOATREAL_EQUAL(expected_dist, dist);
  CHECK(result);

