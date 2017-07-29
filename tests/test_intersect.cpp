

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
  
  vfloat4 expected_dist(4.0);
  result = intersectBox(n, r, zero, inf, dist);
  CHECK(all(expected_dist==dist));
  std::cout << result << std::endl;
  CHECK(result);

  r = TravRay(org,-dir);
  result = intersectBox(n, r, zero, inf, dist);
  std::cout << result << std::endl;
  CHECK(!result);
}
