
#include "Ray.h"
#include "testutil.hpp"
#include "Node.h"
#include "vfloat.h"

void test_intersect();
void test_parallel_hits();

int main (int argc, char** argv) {

  test_intersect();
  test_parallel_hits();
  
  return 0;
}


void test_intersect() {

  AABB box(0.0, 0.0, 0.0,
	   5.0, 5.0, 5.0);

  AANode n = AANode();

  n.setBounds(box);
  
  Vec3fa org = Vec3fa( 9.0, 3.0, 3.0);
  Vec3fa dir = Vec3fa(-1.0, 0.0, 0.0);

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
  float diag = Vec3fa(1.0, 1.0, 1.0).length();

  AANode man_n(lower_x, upper_x,
	       lower_y, upper_y,
	       lower_z, upper_z);

  // intersect lowest box only
  org = Vec3fa(-0.5, -0.5, -0.5);
  // ray goes through low and high corner of each box
  dir = Vec3fa(1.0, 1.0, 1.0);
  dir.normalize();
  r = TravRay(org, dir);
  vfloat4 len = vfloat4(diag);
  float eps = 1.0e-6;

  
  result = intersectBox(man_n, r, z, len, dist);
  // should only hit the first box
  size_t expected_result = 1;
  CHECK_EQUAL(expected_result, result);
  CHECK_REAL_EQUAL(0.5f*diag, dist[0], eps);
  
  //extend the length of this ray
  len = vfloat4(1.5f*diag);
  result = intersectBox(man_n, r, z, len, dist);
  //should hit two boxes
  expected_result = 3;
  CHECK_EQUAL(expected_result, result);
  CHECK_REAL_EQUAL(0.5f*diag, dist[0], eps);
  CHECK_REAL_EQUAL(1.5f*diag, dist[1], eps);

  //extend the length of this ray
  len = vfloat4(2.5*diag);
  result = intersectBox(man_n, r, z, len, dist);
  //should hit three boxes
  expected_result = 7;
  CHECK_EQUAL(expected_result, result);
  CHECK_REAL_EQUAL(0.5f*diag, dist[0], eps);
  CHECK_REAL_EQUAL(1.5f*diag, dist[1], eps);
  CHECK_REAL_EQUAL(2.5f*diag, dist[2], eps);

  //extend the length of this ray
  len = vfloat4(3.5*diag);
  result = intersectBox(man_n, r, z, len, dist);
  //should hit three boxes STILL, last box is a point
  expected_result = 7;
  CHECK_EQUAL(expected_result, result);
  CHECK_REAL_EQUAL(0.5f*diag, dist[0], eps);
  CHECK_REAL_EQUAL(1.5f*diag, dist[1], eps);
  CHECK_REAL_EQUAL(2.5f*diag, dist[2], eps);

  //extend the length of this ray
  len = vfloat4(inf);
  result = intersectBox(man_n, r, z, len, dist);
  // should hit all boxes (including the point-box at the end)
  expected_result = 15;
  CHECK_EQUAL(expected_result, result);

  // setup a glancing hit with the lowest box
  org = Vec3fa(-0.5, -0.5, -0.5);
  dir = Vec3fa(0.0, 0.0, 1.0) - org;
  float exp_dist = dir.length();
  dir.normalize();

  // construct a new traversal ray
  r = TravRay(org, dir);

  // intersect ray of infinite length
  result = intersectBox(man_n, r, z, i, dist);
  // should only intersect the first box
  expected_result = 1;
  CHECK_EQUAL(expected_result, result);
  CHECK_REAL_EQUAL(exp_dist, dist[0], eps);
  
  // setup parallel hit
  org = Vec3fa(-0.5, 0.5, 1.0);
  dir = Vec3fa( 1.0, 0.0, 0.0);
  dir.normalize();

  r = TravRay(org, dir);
  // intersect ray of infinite length
  result = intersectBox(man_n, r, z, i, dist);
  // no hit should be found. ray is coincident to upper bounding plane
  expected_result = 0;
  CHECK_EQUAL(expected_result, result);

  // setup parallel hit
  org = Vec3fa(-0.5, 0.5, 0.0);
  dir = Vec3fa( 1.0, 0.0, 0.0);
  dir.normalize();

  r = TravRay(org, dir);
  // intersect ray of infinite length
  result = intersectBox(man_n, r, z, i, dist);
  // should intersect the box, ray is conincident with lower bounding plane
  expected_result = 1;
  CHECK_EQUAL(expected_result, result);
  exp_dist = 0.5;
  CHECK_REAL_EQUAL(exp_dist, dist[0], eps);
}

void test_parallel_hits() {
  // define a "stack" of unit boxes 1D in the y-z plane
  vfloat4 lower_x(0.0, 0.0, 0.0, 0.0);
  vfloat4 upper_x(1.0, 1.0, 1.0, 1.0);
  
  vfloat4 lower_y(0.0, 1.0, 1.0, 0.0);
  vfloat4 upper_y(1.0, 2.0, 2.0, 1.0);

  vfloat4 lower_z(0.0, 0.0, 1.0, 1.0);
  vfloat4 upper_z(1.0, 1.0, 2.0, 2.0);

  AANode n(lower_x, upper_x, lower_y, upper_y, lower_z, upper_z);

  // create ray aligned between boxes 1 & 2
  // y coordinate of ray coincident with box bounds
  Vec3fa org(2.0, 1.0, 0.5);
  Vec3fa dir(-1.0, 0.0, 0.0);
  TravRay r(org, dir);

  vfloat4 z(zero), i(inf);
  vfloat4 dist;
  float eps = 1e-6;
  
  int result = intersectBox(n, r, z, i, dist);
  // we expect to hit box 2. it is higher in the y coordinate
  int expected_result = 2;
  float expected_dist = 1.0;
  CHECK_EQUAL(expected_result, result);
  CHECK_REAL_EQUAL(expected_dist, dist[1], eps);

  // move ray origin - fire between boxes 2 & 3
  org = Vec3fa(2.0, 1.5, 1.0);
  r = TravRay(org, dir);
  result = intersectBox(n, r, z, i, dist);
  // expected to hit box higher in z coordinate (box 3)
  expected_result = 4;
  expected_dist = 1.0;
  CHECK_EQUAL(expected_result, result);
  CHECK_REAL_EQUAL(expected_dist, dist[2], eps);

  // move ray origin - fire between boxes 3 & 4
  org = Vec3fa(2.0, 1.0, 1.5);
  r = TravRay(org, dir);
  result = intersectBox(n, r, z, i, dist);
  // expected to hit box higher in y coordinate (box 3)
  expected_result = 4;
  expected_dist = 1.0;
  CHECK_EQUAL(expected_result, result);
  CHECK_REAL_EQUAL(expected_dist, dist[2], eps);

  // move ray origin - fire between boxes 2 & 3 (and between 1 & 4)
  // now z coordinate of ray is coincident with box planes
  org = Vec3fa(0.5, 3.0, 1.0);
  dir = Vec3fa(0.0, -1.0, 0.0);
  r = TravRay(org, dir);
  result = intersectBox(n, r, z, i, dist);
  // expected to hit boxes higher in z coordinates (boxes 3 & 4)
  expected_result = 12;
  expected_dist = 1.0;
  CHECK_EQUAL(expected_result, result);
  CHECK_REAL_EQUAL(expected_dist, dist[2], eps);
  expected_dist = 2.0;
  CHECK_REAL_EQUAL(expected_dist, dist[3], eps);
  
  // move ray - fire along the meeting edge of all four boxes
  // y & z coordinates of ray coincident with all boxes
  org = Vec3fa(2.0, 1.0, 1.0);
  dir = Vec3fa(-1.0, 0.0, 0.0);
  r = TravRay(org, dir);
  result = intersectBox(n, r, z, i, dist);
  // expected to hit box higher in BOTH the y & z coordinates (box 3)
  expected_result = 4;
  expected_dist = 1.0;
  CHECK_EQUAL(expected_result, result);
  CHECK_REAL_EQUAL(expected_dist, dist[2], eps);

}

