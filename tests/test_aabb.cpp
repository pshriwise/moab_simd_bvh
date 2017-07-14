#include <stdio.h>

#include "AABB.h"
#include "testutil.hpp"

int main( int argc, char** argv) {

  float x_min = 0.0, y_min = 1.0, z_min = 2.0,
        x_max = 3.0, y_max = 4.0, z_max = 5.0;

  float test_extents[6] = {x_min, y_min, z_min, x_max, y_max, z_max};
  float test_lower[3] = {x_min, y_min, z_min};
  float test_upper[3] = {x_max, y_max, z_max};

  AABB box = AABB(test_extents);

  CHECK_VECREAL_EQUAL(Vec3f(test_lower), box.lower, 0.0);
  CHECK_VECREAL_EQUAL(Vec3f(test_upper), box.upper, 0.0);

  AABB box1 = box;

  CHECK_VECREAL_EQUAL(box.lower, box1.lower, 0.0);
  CHECK_VECREAL_EQUAL(box.upper, box1.upper, 0.0);

  CHECK_VECREAL_EQUAL(Vec3f(test_lower), box1.lower, 0.0);
  CHECK_VECREAL_EQUAL(Vec3f(test_upper), box1.upper, 0.0);


  AABB box2 = AABB(x_min, y_min, z_min, x_max, y_max, z_max);
  
  CHECK_REAL_EQUAL(x_min, box2.lower[0], 0.0);
  CHECK_REAL_EQUAL(y_min, box2.lower[1], 0.0);
  CHECK_REAL_EQUAL(z_min, box2.lower[2], 0.0);
  CHECK_REAL_EQUAL(x_max, box2.upper[0], 0.0);
  CHECK_REAL_EQUAL(y_max, box2.upper[1], 0.0);
  CHECK_REAL_EQUAL(z_max, box2.upper[2], 0.0);
		   
  AABB empty = AABB();

  return 0;

}
