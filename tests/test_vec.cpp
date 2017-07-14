
#include <stdio.h>

#include "Vec3.h"
#include "testutil.hpp"

int main( int argc, char** argv) {

  float x = 0.0, y = 1.0 , z = 2.0;

  Vec3f v = Vec3f(x,y,z);

  CHECK_REAL_EQUAL(x, v.x, 0.);
  CHECK_REAL_EQUAL(y, v.y, 0.);
  CHECK_REAL_EQUAL(z, v.z, 0.);

  CHECK_REAL_EQUAL(x, v[0], 0.);
  CHECK_REAL_EQUAL(y, v[1], 0.);
  CHECK_REAL_EQUAL(z, v[2], 0.);
  
  Vec3f v1 = v;

  CHECK_REAL_EQUAL(x, v1.x, 0.);
  CHECK_REAL_EQUAL(y, v1.y, 0.);
  CHECK_REAL_EQUAL(z, v1.z, 0.);

  Vec3f v2 = Vec3f(v);

  CHECK_REAL_EQUAL(x, v1.x, 0.);
  CHECK_REAL_EQUAL(y, v1.y, 0.);
  CHECK_REAL_EQUAL(z, v1.z, 0.);

  Vec3f empty = Vec3f();
  
  return 0;

}

