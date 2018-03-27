
#include <stdio.h>
#include "Vec3da.h"
#include "testutil.hpp"

void constructor_tests();
void mask_tests();
void all_tests();
void min_max_tests();
void unary_op_tests();
void binary_op_tests();
void convenience_method_tests();

int main( int argc, char** argv) {

  // test Vec3da constructors
  constructor_tests();
  // test all for vec3fa mask
  all_tests();
  // test min max methods
  min_max_tests();
  // testing unary operators of Vec3da
  unary_op_tests();
  // testing binary operators of Vec3da
  binary_op_tests();
  // tests several convenience methods of Vec3da
  convenience_method_tests();
  
  return 0;
}

void constructor_tests() {
  // test construction values
  double x = 0.0, y = 1.0 , z = 2.0;
  // create Vec3da
  Vec3da v = Vec3da(x,y,z);
  // test Vec3da values by attribute
  CHECK_REAL_EQUAL(x, v.x, 0.0);
  CHECK_REAL_EQUAL(y, v.y, 0.0);
  CHECK_REAL_EQUAL(z, v.z, 0.0);

  // test values using iterator
  CHECK_REAL_EQUAL(x, v[0], 0.0);
  CHECK_REAL_EQUAL(y, v[1], 0.0);
  CHECK_REAL_EQUAL(z, v[2], 0.0);

  // test copy constructor
  Vec3da v1 = v;
  CHECK_REAL_EQUAL(x, v1.x, 0.0);
  CHECK_REAL_EQUAL(y, v1.y, 0.0);
  CHECK_REAL_EQUAL(z, v1.z, 0.0);

  // test another form of copy constructor
  Vec3da v2 = Vec3da(v);
  CHECK_REAL_EQUAL(x, v1.x, 0.0);
  CHECK_REAL_EQUAL(y, v1.y, 0.0);
  CHECK_REAL_EQUAL(z, v1.z, 0.0);

  // test empty constructor
  Vec3da empty = Vec3da();
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

  Vec3da vec1 = Vec3da(1.0,2.0,3.0);

  Vec3da vec2 = Vec3da(-1.0,4.0,-6.0);

  Vec3da expected_min_result, min_result;
  expected_min_result = Vec3da(vec2[0],vec1[1],vec2[2]);
  min_result = min(vec1,vec2);
  CHECK_VECREAL_EQUAL(expected_min_result, min_result);

  Vec3da expected_max_result, max_result;
  expected_max_result = Vec3da(vec1[0],vec2[1],vec1[2]);
  max_result = max(vec1,vec2);
  CHECK_VECREAL_EQUAL(expected_max_result, max_result);

}

void unary_op_tests() {

  Vec3da a = Vec3da(1.0, 2.0, 3.0);
  
  Vec3da expected_neg_result(-1.0, -2.0, -3.0);

  CHECK_VECREAL_EQUAL(expected_neg_result,-a);

  Vec3da expected_pos_result(1.0, 2.0, 3.0);

  CHECK_VECREAL_EQUAL(expected_pos_result, +a);

  double mult_val = 2.0;
  
  Vec3da expected_mult_result(2.0, 4.0, 6.0);

  a *= mult_val;
  CHECK_VECREAL_EQUAL(expected_mult_result, a);
  a /= mult_val;

  double div_val = 2.0;

  Vec3da expected_div_result(0.5, 1.0, 1.5);

  a /= div_val;
  CHECK_VECREAL_EQUAL(expected_div_result, a);
  a *= div_val;

  double add_val = 2.0;
  
  Vec3da expected_add_result(3.0, 4.0, 5.0);

  a += add_val;
  CHECK_VECREAL_EQUAL(expected_add_result, a);
  a -= add_val;

  double sub_val = 2.0;
  
  Vec3da expected_sub_result(-1.0, 0.0, 1.0);

  a -= sub_val;
  CHECK_VECREAL_EQUAL(expected_sub_result, a);
  a += sub_val;

}

void binary_op_tests() {

  Vec3da a = Vec3da(1.0, 2.0, 3.0);
  Vec3da b = Vec3da(3.0, 2.0, 1.0);

  Vec3da expected_add_result(4.0, 4.0, 4.0);
  CHECK_VECREAL_EQUAL(expected_add_result, a+b);

  Vec3da expected_sub_result(-2.0, 0.0, 2.0);
  CHECK_VECREAL_EQUAL(expected_sub_result, a-b);

  Vec3da expected_mult_result(3.0, 4.0, 3.0);
  CHECK_VECREAL_EQUAL(expected_mult_result,a*b);

  Vec3da expected_div_result((1.0/3.0), 1.0, 3.0);
  CHECK_VECREAL_EQUAL(expected_div_result,a/b);

  // Scalar Tests

  double val = 10.0;

  expected_mult_result = Vec3da(10.0, 20.0, 30.0);
  CHECK_VECREAL_EQUAL(expected_mult_result,val*a);
  CHECK_VECREAL_EQUAL(expected_mult_result,a*val);

  expected_div_result = Vec3da(0.1, 0.2, 0.3);
  CHECK_VECREAL_EQUAL(expected_div_result, a/val);
  expected_div_result = Vec3da((10.0/1.0), (10.0/2.0), (10.0/3.0));
  CHECK_VECREAL_EQUAL(expected_div_result, val/a);

}

void convenience_method_tests() {

  Vec3da test_vec(3.0,4.0,-2.0);
  double expected_result = -24.0;
  CHECK_REAL_EQUAL(expected_result, reduce_mul(test_vec), 0.0);

  expected_result = 5.0;
  CHECK_REAL_EQUAL(expected_result, reduce_add(test_vec), 0.0);

  expected_result = -2.0;
  CHECK_REAL_EQUAL(expected_result, reduce_min(test_vec), 0.0);

  expected_result = 4.0;
  CHECK_REAL_EQUAL(expected_result, reduce_max(test_vec), 0.0);

  Vec3da test_vec1 = Vec3da(4.0,3.0,2.0);
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

  test_vec = Vec3da(5.0, 5.0, 5.0);
  double expected_length = sqrt(75.0);
  CHECK_REAL_EQUAL(expected_length, test_vec.length(), 0.0);

  // make into a vector of unit length
  test_vec.normalize();
  expected_length = 1.0;
  CHECK_REAL_EQUAL(expected_length, test_vec.length(), ulp);  
}
