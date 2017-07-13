#include <stdio.h>

#include "AABB.h"
#include "testutil.hpp"

int main( int argc, char** argv) {

  float x_min = 0.0, y_min = 1.0, z_min = 2.0,
        x_max = 3.0, y_max = 4.0, z_max = 5.0;

  float test_extents[6] = {x_min, y_min, z_min, x_max, y_max, z_max};

  AABB box = AABB(test_extents);

  CHECK_ARRAYS_EQUAL(test_extents, 6, box.extents, 6);

  AABB box1 = box;

  CHECK_ARRAYS_EQUAL(test_extents, 6, box1.extents, 6);

  AABB box2 = AABB(x_min, y_min, z_min, x_max, y_max, z_max);
  
  CHECK_ARRAYS_EQUAL(test_extents, 6, box2.extents, 6);
  
  AABB empty = AABB();
  
  return 0;

}
