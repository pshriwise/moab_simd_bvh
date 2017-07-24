
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

  Vec3f a = Vec3f(2.0,3.0,2.0);
  Vec3f b = Vec3f(1.0,3.0,4.0);

  Vec3<bool> expected_result = Vec3<bool>(true, true, false);
  
  Vec3<bool> result = ge_mask(a,b);

  CHECK_VECREAL_EQUAL(expected_result, result);

  expected_result = Vec3<bool>(false, true, true);

  result = le_mask(a,b);

  CHECK_VECREAL_EQUAL(expected_result, result);

  //test for all true
  Vec3<bool> all_test(true,true,true);
  CHECK(all(all_test));

  // flip one entry
  all_test[0] = false;
  CHECK(!all(all_test));
  // and another
  all_test[1] = false;
  CHECK(!all(all_test));
  // and the last one
  all_test[2] = false;
  CHECK(!all(all_test));
  
  return 0;

}

