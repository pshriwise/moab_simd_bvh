
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
  size_t result;

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

  // test node with different bounds for each box
  vfloat4 lower_x(0.0, 1.0, 2.0, 5.0);
  vfloat4 upper_x(1.0, 2.0, 3.0, 5.0);
  vfloat4 lower_y(0.0, 1.0, 2.0, 5.0);
  vfloat4 upper_y(1.0, 2.0, 3.0, 5.0);
  vfloat4 lower_z(0.0, 1.0, 2.0, 5.0);
  vfloat4 upper_z(1.0, 2.0, 3.0, 5.0);

  // longest diagonal of unit box
  float diag = Vec3f(1.0, 1.0, 1.0).length();

  AANode man_n(lower_x, upper_x,
	       lower_y, upper_y,
	       lower_z, upper_z);

  // intersect lowest box only
  org = Vec3f(-0.5, -0.5, -0.5);
  // ray goes through low and high corner of each box
  dir = Vec3f(1.0, 1.0, 1.0);
  dir.normalize();
  r = TravRay(org, dir);
  vfloat4 len = vfloat4(diag);
  float eps = 1.0e-6;

  
  result = intersectBox(man_n, r, z, len, dist);
  // should only hit the first box
  size_t expected_result = 1;
  CHECK_EQUAL(expected_result, result);
  CHECK_REAL_EQUAL(0.5*diag, dist[0], eps);
  
  //extend the length of this ray
  len = vfloat4(1.5*diag);
  result = intersectBox(man_n, r, z, len, dist);
  //should hit two boxes
  expected_result = 3;
  CHECK_EQUAL(expected_result, result);
  CHECK_REAL_EQUAL(0.5*diag, dist[0], eps);
  CHECK_REAL_EQUAL(1.5*diag, dist[1], eps);

  //extend the length of this ray
  len = vfloat4(2.5*diag);
  result = intersectBox(man_n, r, z, len, dist);
  //should hit three boxes
  expected_result = 7;
  CHECK_EQUAL(expected_result, result);
  CHECK_REAL_EQUAL(0.5*diag, dist[0], eps);
  CHECK_REAL_EQUAL(1.5*diag, dist[1], eps);
  CHECK_REAL_EQUAL(2.5*diag, dist[2], eps);

  //extend the length of this ray
  len = vfloat4(3.5*diag);
  result = intersectBox(man_n, r, z, len, dist);
  //should hit three boxes STILL, last box is a point
  expected_result = 7;
  CHECK_EQUAL(expected_result, result);
  CHECK_REAL_EQUAL(0.5*diag, dist[0], eps);
  CHECK_REAL_EQUAL(1.5*diag, dist[1], eps);
  CHECK_REAL_EQUAL(2.5*diag, dist[2], eps);

  //extend the length of this ray
  len = vfloat4(inf);
  result = intersectBox(man_n, r, z, len, dist);
  // should hit all boxes (including the point-box at the end)
  expected_result = 15;
  CHECK_EQUAL(expected_result, result);

}
