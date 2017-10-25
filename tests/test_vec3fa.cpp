
#include <stdio.h>
#include "Vec3fa.h"
#include "testutil.hpp"

void constructor_tests();
void mask_tests();
void all_tests();
void min_max_tests();
void unary_op_tests();
void binary_op_tests();
void convenience_method_tests();

int main( int argc, char** argv) {

  // test Vec3fa constructors
  constructor_tests();
  // test all for vec3fa mask
  all_tests();
  // test min max methods
  min_max_tests();
  // testing unary operators of Vec3fa
  unary_op_tests();
  // testing binary operators of Vec3fa
  binary_op_tests();
  // tests several convenience methods of Vec3fa
  convenience_method_tests();
  
  return 0;
}

void constructor_tests() {
  // test construction values
  float x = 0.0, y = 1.0 , z = 2.0;
  // create Vec3fa
  Vec3fa v = Vec3fa(x,y,z);
  // test Vec3fa values by attribute
  CHECK_REAL_EQUAL(x, v.x, 0.);
  CHECK_REAL_EQUAL(y, v.y, 0.);
  CHECK_REAL_EQUAL(z, v.z, 0.);

  // test values using iterator
  CHECK_REAL_EQUAL(x, v[0], 0.);
  CHECK_REAL_EQUAL(y, v[1], 0.);
  CHECK_REAL_EQUAL(z, v[2], 0.);

  // test copy constructor
  Vec3fa v1 = v;
  CHECK_REAL_EQUAL(x, v1.x, 0.);
  CHECK_REAL_EQUAL(y, v1.y, 0.);
  CHECK_REAL_EQUAL(z, v1.z, 0.);

  // test another form of copy constructor
  Vec3fa v2 = Vec3fa(v);
  CHECK_REAL_EQUAL(x, v1.x, 0.);
  CHECK_REAL_EQUAL(y, v1.y, 0.);
  CHECK_REAL_EQUAL(z, v1.z, 0.);

  // test empty constructor
  Vec3fa empty = Vec3fa();
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

  Vec3fa vec1 = Vec3fa(1.0,2.0,3.0);

  Vec3fa vec2 = Vec3fa(-1.0,4.0,-6.0);

  Vec3fa expected_min_result, min_result;
  expected_min_result = Vec3fa(vec2[0],vec1[1],vec2[2]);
  min_result = min(vec1,vec2);
  CHECK_VECREAL_EQUAL(expected_min_result, min_result);

  Vec3fa expected_max_result, max_result;
  expected_max_result = Vec3fa(vec1[0],vec2[1],vec1[2]);
  max_result = max(vec1,vec2);
  CHECK_VECREAL_EQUAL(expected_max_result, max_result);

}

void unary_op_tests() {

  Vec3fa a = Vec3fa(1.0, 2.0, 3.0);
  
  Vec3fa expected_neg_result(-1.0, -2.0, -3.0);

  CHECK_VECREAL_EQUAL(expected_neg_result,-a);

  Vec3fa expected_pos_result(1.0, 2.0, 3.0);

  CHECK_VECREAL_EQUAL(expected_pos_result, +a);
  
}

void binary_op_tests() {

  Vec3fa a = Vec3fa(1.0, 2.0, 3.0);
  Vec3fa b = Vec3fa(3.0, 2.0, 1.0);

  Vec3fa expected_add_result(4.0, 4.0, 4.0);
  CHECK_VECREAL_EQUAL(expected_add_result, a+b);

  Vec3fa expected_sub_result(-2.0, 0.0, 2.0);
  CHECK_VECREAL_EQUAL(expected_sub_result, a-b);

  Vec3fa expected_mult_result(3.0, 4.0, 3.0);
  CHECK_VECREAL_EQUAL(expected_mult_result,a*b);

  Vec3fa expected_div_result((1.0/3.0), 1.0, 3.0);
  CHECK_VECREAL_EQUAL(expected_div_result,a/b);

  // Scalar Tests

  float val = 10.0;

  expected_mult_result = Vec3fa(10.0, 20.0, 30.0);
  CHECK_VECREAL_EQUAL(expected_mult_result,val*a);
  CHECK_VECREAL_EQUAL(expected_mult_result,a*val);

  expected_div_result = Vec3fa(0.1, 0.2, 0.3);
  CHECK_VECREAL_EQUAL(expected_div_result, a/val);
  expected_div_result = Vec3fa((10.0/1.0), (10.0/2.0), (10.0/3.0));
  CHECK_VECREAL_EQUAL(expected_div_result, val/a);

}

void convenience_method_tests() {

  Vec3fa test_vec(3.0,4.0,-2.0);
  float expected_result = -24.0;
  CHECK_REAL_EQUAL(expected_result, reduce_mul(test_vec), 0.0);

  expected_result = 5.0;
  CHECK_REAL_EQUAL(expected_result, reduce_add(test_vec), 0.0);

  expected_result = -2.0;
  CHECK_REAL_EQUAL(expected_result, reduce_min(test_vec), 0.0);

  expected_result = 4.0;
  CHECK_REAL_EQUAL(expected_result, reduce_max(test_vec), 0.0);

  Vec3fa test_vec1 = Vec3fa(4.0,3.0,2.0);
  expected_result = 24.0;
  CHECK_REAL_EQUAL(expected_result, reduce_mul(test_vec1), 0.0);

  expected_result = 9.0;
  CHECK_REAL_EQUAL(expected_result, reduce_add(test_vec1), 0.0);

  expected_result = 2.0;
  CHECK_REAL_EQUAL(expected_result, reduce_min(test_vec1), 0.0);

  expected_result = 4.0;
  CHECK_REAL_EQUAL(expected_result, reduce_max(test_vec1), 0.0);

  expected_result = 26.0;
  CHECK_REAL_EQUAL(expected_result, halfArea(test_vec1), 0.0);

  test_vec = Vec3fa(5.0, 5.0, 5.0);
  float expected_length = sqrt(75.0);
  CHECK_REAL_EQUAL(expected_length, test_vec.length(), 0.0);

  // make into a vector of unit length
  test_vec.normalize();
  expected_length = 1.0;
  CHECK_REAL_EQUAL(expected_length, test_vec.length(), ulp);  
}
