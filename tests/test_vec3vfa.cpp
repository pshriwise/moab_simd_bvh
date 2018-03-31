
#include <stdio.h>
#include "Vec3vfa.h"
#include "testutil.hpp"

#define EPS 0.0f

void constructor_tests();
void min_max_tests();
void unary_op_tests();
void binary_op_tests();
void convenience_method_tests();

int main(int argc, char** argv) {

  constructor_tests();
  min_max_tests();
  unary_op_tests();
  binary_op_tests();
  convenience_method_tests();
  
  return 0;
}

void constructor_tests() {
  // test construction values
  float x = 0.0, y = 1.0 , z = 2.0;
  // create Vec3vfa
  Vec3vfa v = Vec3vfa(x,y,z);

  vfloat4 xv(x), yv(y), zv(z);
  // test Vec3vfa values by attribute
  CHECK_VFLOATREAL_EQUAL(xv, v.x);
  CHECK_VFLOATREAL_EQUAL(yv, v.y);
  CHECK_VFLOATREAL_EQUAL(zv, v.z);

  // test values using iterator
  CHECK_VFLOATREAL_EQUAL(xv, v[0]);
  CHECK_VFLOATREAL_EQUAL(yv, v[1]);
  CHECK_VFLOATREAL_EQUAL(zv, v[2]);

  // test copy constructor
  Vec3vfa v1 = v;
  CHECK_VFLOATREAL_EQUAL(xv, v1.x);
  CHECK_VFLOATREAL_EQUAL(yv, v1.y);
  CHECK_VFLOATREAL_EQUAL(zv, v1.z);

  // test another form of copy constructor
  Vec3vfa v2 = Vec3vfa(v);
  CHECK_VFLOATREAL_EQUAL(xv, v1.x);
  CHECK_VFLOATREAL_EQUAL(yv, v1.y);
  CHECK_VFLOATREAL_EQUAL(zv, v1.z);

  // test empty constructor
  Vec3vfa empty = Vec3vfa();
}


void min_max_tests() {

  Vec3vfa vec1 = Vec3vfa(1.0,2.0,3.0);

  Vec3vfa vec2 = Vec3vfa(-1.0,4.0,-6.0);

  Vec3vfa expected_min_result, min_result;
  expected_min_result = Vec3vfa(vec2[0],vec1[1],vec2[2]);
  min_result = min(vec1,vec2);
  CHECK_VFLOATREAL_EQUAL(expected_min_result.x, min_result.x);
  CHECK_VFLOATREAL_EQUAL(expected_min_result.y, min_result.y);
  CHECK_VFLOATREAL_EQUAL(expected_min_result.z, min_result.z);

  Vec3vfa expected_max_result, max_result;
  expected_max_result = Vec3vfa(vec1[0],vec2[1],vec1[2]);
  max_result = max(vec1,vec2);
  CHECK_VFLOATREAL_EQUAL(expected_max_result.x, max_result.x);
  CHECK_VFLOATREAL_EQUAL(expected_max_result.y, max_result.y);
  CHECK_VFLOATREAL_EQUAL(expected_max_result.z, max_result.z);

}

void unary_op_tests() {

  Vec3vfa a = Vec3vfa(1.0, 2.0, 3.0);
  
  Vec3vfa expected_neg_result(-1.0, -2.0, -3.0);

  CHECK_VFLOATREAL_EQUAL(expected_neg_result.x,-a.x);
  CHECK_VFLOATREAL_EQUAL(expected_neg_result.y,-a.y);
  CHECK_VFLOATREAL_EQUAL(expected_neg_result.z,-a.z);

  Vec3vfa expected_pos_result(1.0, 2.0, 3.0);

  CHECK_VFLOATREAL_EQUAL(expected_pos_result.x, +a.x);
  CHECK_VFLOATREAL_EQUAL(expected_pos_result.y, +a.y);
  CHECK_VFLOATREAL_EQUAL(expected_pos_result.z, +a.z);

  float mult_val = 2.0;
  
  Vec3vfa expected_mult_result(2.0, 4.0, 6.0);

  a *= mult_val;
  CHECK_VFLOATREAL_EQUAL(expected_mult_result.x, a.x);
  CHECK_VFLOATREAL_EQUAL(expected_mult_result.y, a.y);
  CHECK_VFLOATREAL_EQUAL(expected_mult_result.z, a.z);
  a /= mult_val;

  float div_val = 2.0;

  Vec3vfa expected_div_result(0.5, 1.0, 1.5);

  a /= div_val;
  CHECK_VFLOATREAL_EQUAL(expected_div_result.x, a.x);
  CHECK_VFLOATREAL_EQUAL(expected_div_result.y, a.y);
  CHECK_VFLOATREAL_EQUAL(expected_div_result.z, a.z);
  a *= div_val;

  float add_val = 2.0;
  
  Vec3vfa expected_add_result(3.0, 4.0, 5.0);

  a += add_val;
  CHECK_VFLOATREAL_EQUAL(expected_add_result.x, a.x);
  CHECK_VFLOATREAL_EQUAL(expected_add_result.y, a.y);
  CHECK_VFLOATREAL_EQUAL(expected_add_result.z, a.z);
  a -= add_val;

  float sub_val = 2.0;
  
  Vec3vfa expected_sub_result(-1.0, 0.0, 1.0);

  a -= sub_val;
  CHECK_VFLOATREAL_EQUAL(expected_sub_result.x, a.x);
  CHECK_VFLOATREAL_EQUAL(expected_sub_result.y, a.y);
  CHECK_VFLOATREAL_EQUAL(expected_sub_result.z, a.z);
  a += sub_val;
  
}

void binary_op_tests() {

  Vec3vfa a = Vec3vfa(1.0, 2.0, 3.0);
  Vec3vfa b = Vec3vfa(3.0, 2.0, 1.0);

  Vec3vfa expected_add_result(4.0, 4.0, 4.0);
  CHECK_VFLOATREAL_EQUAL(expected_add_result.x, (a+b).x);
  CHECK_VFLOATREAL_EQUAL(expected_add_result.y, (a+b).y);
  CHECK_VFLOATREAL_EQUAL(expected_add_result.z, (a+b).z);

  Vec3vfa expected_sub_result(-2.0, 0.0, 2.0);
  CHECK_VFLOATREAL_EQUAL(expected_sub_result.x, (a-b).x);
  CHECK_VFLOATREAL_EQUAL(expected_sub_result.y, (a-b).y);
  CHECK_VFLOATREAL_EQUAL(expected_sub_result.z, (a-b).z);

  Vec3vfa expected_mult_result(3.0, 4.0, 3.0);
  CHECK_VFLOATREAL_EQUAL(expected_mult_result.x, (a*b).x);
  CHECK_VFLOATREAL_EQUAL(expected_mult_result.y, (a*b).y);
  CHECK_VFLOATREAL_EQUAL(expected_mult_result.z, (a*b).z);

  Vec3vfa expected_div_result((1.0/3.0), 1.0, 3.0);
  CHECK_VFLOATREAL_EQUAL(expected_div_result.x, (a/b).x);
  CHECK_VFLOATREAL_EQUAL(expected_div_result.y, (a/b).y);
  CHECK_VFLOATREAL_EQUAL(expected_div_result.z, (a/b).z);

  // Scalar Tests

  float val = 10.0;

  expected_mult_result = Vec3vfa(10.0, 20.0, 30.0);
  CHECK_VFLOATREAL_EQUAL(expected_mult_result.x,val*a.x);
  CHECK_VFLOATREAL_EQUAL(expected_mult_result.y,val*a.y);
  CHECK_VFLOATREAL_EQUAL(expected_mult_result.z,val*a.z);
  
  CHECK_VFLOATREAL_EQUAL(expected_mult_result.x,a.x*val);
  CHECK_VFLOATREAL_EQUAL(expected_mult_result.y,a.y*val);
  CHECK_VFLOATREAL_EQUAL(expected_mult_result.z,a.z*val);

  expected_div_result = Vec3vfa(0.1, 0.2, 0.3);
  CHECK_VFLOATREAL_EQUAL(expected_div_result.x, a.x/val);
  CHECK_VFLOATREAL_EQUAL(expected_div_result.y, a.y/val);
  CHECK_VFLOATREAL_EQUAL(expected_div_result.z, a.z/val);
  
  expected_div_result = Vec3vfa((10.0/1.0), (10.0/2.0), (10.0/3.0));
  CHECK_VFLOATREAL_EQUAL(expected_div_result.x, val/a.x);
  CHECK_VFLOATREAL_EQUAL(expected_div_result.y, val/a.y);
  CHECK_VFLOATREAL_EQUAL(expected_div_result.z, val/a.z);

}

void convenience_method_tests() {

  Vec3vfa test_vec(3.0,4.0,-2.0);
  float expected_result = -24.0;
  vfloat4 expected_result_vec = vfloat4(expected_result);
  CHECK_VFLOATREAL_EQUAL(expected_result_vec, reduce_mul(test_vec));

  expected_result= 5.0;
  expected_result_vec = vfloat4(expected_result);
  CHECK_VFLOATREAL_EQUAL(expected_result_vec, reduce_add(test_vec));

  expected_result = -2.0;
  expected_result_vec = vfloat4(expected_result);
  CHECK_VFLOATREAL_EQUAL(expected_result_vec, reduce_min(test_vec));

  expected_result = 4.0;
  expected_result_vec = vfloat4(expected_result);
  CHECK_VFLOATREAL_EQUAL(expected_result_vec, reduce_max(test_vec));

  Vec3vfa test_vec1 = Vec3vfa(4.0,3.0,2.0);
  expected_result = 24.0;
  expected_result_vec = vfloat4(expected_result);
  CHECK_VFLOATREAL_EQUAL(expected_result_vec, reduce_mul(test_vec1));

  expected_result = 9.0;
  expected_result_vec = vfloat4(expected_result);
  CHECK_VFLOATREAL_EQUAL(expected_result_vec, reduce_add(test_vec1));

  expected_result = 2.0;
  expected_result_vec = vfloat4(expected_result);
  CHECK_VFLOATREAL_EQUAL(expected_result_vec, reduce_min(test_vec1));

  expected_result = 4.0;
  expected_result_vec = vfloat4(expected_result);
  CHECK_VFLOATREAL_EQUAL(expected_result_vec, reduce_max(test_vec1));

  expected_result = 26.0;
  expected_result_vec = vfloat4(expected_result);
  CHECK_VFLOATREAL_EQUAL(expected_result_vec, halfArea(test_vec1));

  test_vec = Vec3vfa(5.0, 5.0, 5.0);
  float expected_length = sqrt(75.0);
  vfloat4 expected_length_vec = vfloat4(expected_length);
  CHECK_VFLOATREAL_EQUAL(expected_length_vec, test_vec.length());

  // make into a vector of unit length
  test_vec.normalize();
  expected_length = 1.0;
  expected_length_vec = vfloat4(expected_length);
  CHECK_VFLOATREAL_EQUAL_TOL(expected_length_vec, test_vec.length(), ulp);  
}
