
#include <stdio.h>
#include "Vec3.h"
#include "testutil.hpp"

void constructor_tests();
void mask_tests();
void all_tests();

int main( int argc, char** argv) {

  // test Vec3 constructors
  constructor_tests();
  // test Vec3 masks
  mask_tests();
  // test all for vec3 mask
  all_tests();
  return 0;
}

void constructor_tests() {
  // test construction values
  float x = 0.0, y = 1.0 , z = 2.0;
  // create Vec3
  Vec3f v = Vec3f(x,y,z);
  // test Vec3 values by attribute
  CHECK_REAL_EQUAL(x, v.x, 0.);
  CHECK_REAL_EQUAL(y, v.y, 0.);
  CHECK_REAL_EQUAL(z, v.z, 0.);

  // test values using iterator
  CHECK_REAL_EQUAL(x, v[0], 0.);
  CHECK_REAL_EQUAL(y, v[1], 0.);
  CHECK_REAL_EQUAL(z, v[2], 0.);

  // test copy constructor
  Vec3f v1 = v;
  CHECK_REAL_EQUAL(x, v1.x, 0.);
  CHECK_REAL_EQUAL(y, v1.y, 0.);
  CHECK_REAL_EQUAL(z, v1.z, 0.);

  // test another form of copy constructor
  Vec3f v2 = Vec3f(v);
  CHECK_REAL_EQUAL(x, v1.x, 0.);
  CHECK_REAL_EQUAL(y, v1.y, 0.);
  CHECK_REAL_EQUAL(z, v1.z, 0.);

  // test empty constructor
  Vec3f empty = Vec3f();
}

void mask_tests() {
  // create two vecs with values
  Vec3f a = Vec3f(2.0,3.0,2.0);
  Vec3f b = Vec3f(1.0,3.0,4.0);

  // the expected greater than mask result for a greater than b
  Vec3<bool> expected_result = Vec3<bool>(true, true, false);
  // get mask
  Vec3<bool> result = ge_mask(a,b);
  // test mask against expected result
  CHECK_VECREAL_EQUAL(expected_result, result);

  // update expected result for a less than b
  expected_result = Vec3<bool>(false, true, true);
  // create mask
  result = le_mask(a,b);
  // test mask against expected result
  CHECK_VECREAL_EQUAL(expected_result, result);

}

void all_tests() {
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
}
