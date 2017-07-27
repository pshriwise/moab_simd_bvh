

#include "AABB.h"
#include "Ray.h"
#include "testutil.hpp"


void test_intersect();

int main (int argc, char** argv) {

  test_intersect();

  return 0;
}


void test_intersect() {

  AABB box(0.0, 0.0, 0.0,
	   5.0, 5.0, 5.0);

  TravRay r(Vec3f( 9.0, 2.5, 2.5), //origin
	    Vec3f(-1.0, 0.0, 0.0));

  
  float dist = 100.0;
  bool result;
  result = intersectBox(box, r, zero, inf, dist);
  std::cout << "Dist: " << dist << std::endl;
  CHECK(result);



}
