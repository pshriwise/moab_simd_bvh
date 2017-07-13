
#include <stdio.h>

#include "Vec3.h"
#include "testutil.hpp"

int main( int argc, char* argv) {

  float x = 0.0, y = 1.0 , z = 2.0;

  Vec3f v = Vec3f(x,y,z);

  CHECK_REAL_EQUAL(x, v.x, 0.);
  CHECK_REAL_EQUAL(y, v.y, 0.);
  CHECK_REAL_EQUAL(z, v.z, 0.);

  return 0;

}

