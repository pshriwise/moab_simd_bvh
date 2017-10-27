
#include <stdio.h>
#include "Vec3.h"
#include "testutil.hpp"

void constructor_tests();
void mask_tests();
void all_tests();
void min_max_tests();
void unary_op_tests();
void binary_op_tests();
void convenience_method_tests();

int main( int argc, char** argv) {

  // test Vec3 constructors
  constructor_tests();
  // test Vec3 masks
  mask_tests();
  // test all for vec3 mask
  all_tests();
  // test min max methods
  min_max_tests();
  // testing unary operators of Vec3
  unary_op_tests();
  // testing binary operators of Vec3
  binary_op_tests();
  // tests several convenience methods of Vec3
  convenience_method_tests();
  
  return 0;
}

void constructor_tests() {
  // test construction values
  float x = 0.0, y = 1.0 , z = 2.0;
  // create Vec3
  Vec3f v = Vec3f(x,y,z);
  // test Vec3 values by attribute
  CHECK_REAL_EQUAL(x, v.x, 0.0f);
  CHECK_REAL_EQUAL(y, v.y, 0.0f);
  CHECK_REAL_EQUAL(z, v.z, 0.0f);

  // test values using iterator
  CHECK_REAL_EQUAL(x, v[0], 0.0f);
  CHECK_REAL_EQUAL(y, v[1], 0.0f);
  CHECK_REAL_EQUAL(z, v[2], 0.0f);

  // test copy constructor
  Vec3f v1 = v;
  CHECK_REAL_EQUAL(x, v1.x, 0.0f);
  CHECK_REAL_EQUAL(y, v1.y, 0.0f);
  CHECK_REAL_EQUAL(z, v1.z, 0.0f);

  // test another form of copy constructor
  Vec3f v2 = Vec3f(v);
  CHECK_REAL_EQUAL(x, v1.x, 0.0f);
  CHECK_REAL_EQUAL(y, v1.y, 0.0f);
  CHECK_REAL_EQUAL(z, v1.z, 0.0f);

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

void min_max_tests() {

  Vec3f vec1 = Vec3f(1.0,2.0,3.0);

  Vec3f vec2 = Vec3f(-1.0,4.0,-6.0);

  Vec3f expected_min_result, min_result;
  expected_min_result = Vec3f(vec2[0],vec1[1],vec2[2]);
  min_result = min(vec1,vec2);
  CHECK_VECREAL_EQUAL(expected_min_result, min_result);

  Vec3f expected_max_result, max_result;
  expected_max_result = Vec3f(vec1[0],vec2[1],vec1[2]);
  max_result = max(vec1,vec2);
  CHECK_VECREAL_EQUAL(expected_max_result, max_result);

}

void unary_op_tests() {

  Vec3f a = Vec3f(1.0, 2.0, 3.0);
  
  Vec3f expected_neg_result(-1.0, -2.0, -3.0);

  CHECK_VECREAL_EQUAL(expected_neg_result,-a);

  Vec3f expected_pos_result(1.0, 2.0, 3.0);

  CHECK_VECREAL_EQUAL(expected_pos_result, +a);
  
}

void binary_op_tests() {

  Vec3f a = Vec3f(1.0, 2.0, 3.0);
  Vec3f b = Vec3f(3.0, 2.0, 1.0);

  Vec3f expected_add_result(4.0, 4.0, 4.0);
  CHECK_VECREAL_EQUAL(expected_add_result, a+b);

  Vec3f expected_sub_result(-2.0, 0.0, 2.0);
  CHECK_VECREAL_EQUAL(expected_sub_result, a-b);

  Vec3f expected_mult_result(3.0, 4.0, 3.0);
  CHECK_VECREAL_EQUAL(expected_mult_result,a*b);

  Vec3f expected_div_result((1.0/3.0), 1.0, 3.0);
  CHECK_VECREAL_EQUAL(expected_div_result,a/b);

  // Scalar Tests

  float val = 10.0;

  expected_mult_result = Vec3f(10.0, 20.0, 30.0);
  CHECK_VECREAL_EQUAL(expected_mult_result,val*a);
  CHECK_VECREAL_EQUAL(expected_mult_result,a*val);

  expected_div_result = Vec3f(0.1, 0.2, 0.3);
  CHECK_VECREAL_EQUAL(expected_div_result, a/val);
  expected_div_result = Vec3f((10.0/1.0), (10.0/2.0), (10.0/3.0));
  CHECK_VECREAL_EQUAL(expected_div_result, val/a);

}

void convenience_method_tests() {

  Vec3f test_vec(3.0,4.0,-2.0);
  float expected_result = -24.0;
  CHECK_REAL_EQUAL(expected_result, reduce_mul(test_vec), 0.0f);

  expected_result = 5.0;
  CHECK_REAL_EQUAL(expected_result, reduce_add(test_vec), 0.0f);

  expected_result = -2.0;
  CHECK_REAL_EQUAL(expected_result, reduce_min(test_vec), 0.0f);

  expected_result = 4.0;
  CHECK_REAL_EQUAL(expected_result, reduce_max(test_vec), 0.0f);

  Vec3f test_vec1 = Vec3f(4.0,3.0,2.0);
  expected_result = 24.0;
  CHECK_REAL_EQUAL(expected_result, reduce_mul(test_vec1), 0.0f);

  expected_result = 9.0;
  CHECK_REAL_EQUAL(expected_result, reduce_add(test_vec1), 0.0f);

  expected_result = 2.0;
  CHECK_REAL_EQUAL(expected_result, reduce_min(test_vec1), 0.0f);

  expected_result = 4.0;
  CHECK_REAL_EQUAL(expected_result, reduce_max(test_vec1), 0.0f);

  expected_result = 26.0;
  CHECK_REAL_EQUAL(expected_result, halfArea(test_vec1), 0.0f);

  test_vec = Vec3f(5.0, 5.0, 5.0);
  float expected_length = sqrt(75.0);
  CHECK_REAL_EQUAL(expected_length, test_vec.length(), 0.0f);

  // make into a vector of unit length
  test_vec.normalize();
  expected_length = 1.0;
  CHECK_REAL_EQUAL(expected_length, test_vec.length(), ulp);  
}
