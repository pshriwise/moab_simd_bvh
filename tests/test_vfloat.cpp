

#include "vfloat.h"
#include "testutil.hpp"
#include "vbool.h"

void test_vfloat4_constructors();
void test_vfloat4_operators();
void test_vfloat4_comparators();
void test_vfloat4_methods();
void test_vfloat4_load_store();

int main( int argc, char** argv ) {

  // test that values are correct after construction
  test_vfloat4_constructors();
  // add, sub, mul, div operator tests
  test_vfloat4_operators();
  // comparator tests
  test_vfloat4_comparators();
  // common but non-standard operation tests
  test_vfloat4_methods();
  // testing store functionality
  test_vfloat4_load_store();
  
  return 0;
}

void test_vfloat4_constructors() {

  float val = 0.0;
  vfloat4 v(val);

  CHECK_REAL_EQUAL(val, v[0], 0.0); 
  CHECK_REAL_EQUAL(val, v[1], 0.0);
  CHECK_REAL_EQUAL(val, v[2], 0.0);
  CHECK_REAL_EQUAL(val, v[3], 0.0);

  float vals[4] = {-6.0, -1.0, 2.0, -3.0};
  
  v = vfloat4(vals[0],vals[1], vals[2], vals[3]);
  
  CHECK_REAL_EQUAL(vals[0], v[0], 0.0); 
  CHECK_REAL_EQUAL(vals[1], v[1], 0.0);
  CHECK_REAL_EQUAL(vals[2], v[2], 0.0);
  CHECK_REAL_EQUAL(vals[3], v[3], 0.0);

  // test copy
  vfloat4 v1 = vfloat4(v);
  
  CHECK_REAL_EQUAL(vals[0], v1[0], 0.0); 
  CHECK_REAL_EQUAL(vals[1], v1[1], 0.0);
  CHECK_REAL_EQUAL(vals[2], v1[2], 0.0);
  CHECK_REAL_EQUAL(vals[3], v1[3], 0.0);
  
  const void* const p = (void*)vals;
  v = vfloat4::load(p);
  
  CHECK_REAL_EQUAL(vals[0], v[0], 0.0); 
  CHECK_REAL_EQUAL(vals[1], v[1], 0.0);
  CHECK_REAL_EQUAL(vals[2], v[2], 0.0);
  CHECK_REAL_EQUAL(vals[3], v[3], 0.0);

}

void test_vfloat4_operators() {
  
  float vals[4] = {-6.0, -1.0, 2.0, -3.0};

  vfloat4 v = vfloat4(vals[0],vals[1], vals[2], vals[3]);
    
  v*=2.0;

  CHECK_REAL_EQUAL(2.*vals[0], v[0], 0.0); 
  CHECK_REAL_EQUAL(2.*vals[1], v[1], 0.0);
  CHECK_REAL_EQUAL(2.*vals[2], v[2], 0.0);
  CHECK_REAL_EQUAL(2.*vals[3], v[3], 0.0);

  v/=2.0;
  
  CHECK_REAL_EQUAL(vals[0], v[0], 0.0); 
  CHECK_REAL_EQUAL(vals[1], v[1], 0.0);
  CHECK_REAL_EQUAL(vals[2], v[2], 0.0);
  CHECK_REAL_EQUAL(vals[3], v[3], 0.0);

  v += 2.0;

  CHECK_REAL_EQUAL(vals[0]+2.0, v[0], 0.0); 
  CHECK_REAL_EQUAL(vals[1]+2.0, v[1], 0.0);
  CHECK_REAL_EQUAL(vals[2]+2.0, v[2], 0.0);
  CHECK_REAL_EQUAL(vals[3]+2.0, v[3], 0.0);

  v -= 2.0;
  
  CHECK_REAL_EQUAL(vals[0], v[0], 0.0); 
  CHECK_REAL_EQUAL(vals[1], v[1], 0.0);
  CHECK_REAL_EQUAL(vals[2], v[2], 0.0);
  CHECK_REAL_EQUAL(vals[3], v[3], 0.0);
  
  vfloat4 a = v*v;

  CHECK_REAL_EQUAL(vals[0]*vals[0], a[0], 0.0); 
  CHECK_REAL_EQUAL(vals[1]*vals[1], a[1], 0.0); 
  CHECK_REAL_EQUAL(vals[2]*vals[2], a[2], 0.0); 
  CHECK_REAL_EQUAL(vals[3]*vals[3], a[3], 0.0); 

  a = v/v;
  
  CHECK_REAL_EQUAL(1.0, a[0], 0.0); 
  CHECK_REAL_EQUAL(1.0, a[1], 0.0); 
  CHECK_REAL_EQUAL(1.0, a[2], 0.0); 
  CHECK_REAL_EQUAL(1.0, a[3], 0.0); 

  
}

void test_vfloat4_methods() {
  float vals[4] = {-6.0, -1.0, 2.0, -3.0};
  float maximum = 2.0, minimum = -6.0;

  vfloat4 v1 = vfloat4(vals[0],vals[1], vals[2], vals[3]);
  
  CHECK_REAL_EQUAL(maximum, max(v1), 0.0);
  CHECK_REAL_EQUAL(minimum, min(v1), 0.0);

  vfloat4 v2 = vfloat4(vals[0],vals[1], vals[2], vals[3]);
  vfloat4 v3 = vfloat4(vals[0],vals[1], vals[2], vals[3]);

  vfloat4 result = madd(v1,v2,v3);
  vfloat4 expected_result = vfloat4(30.0, 0.0, 6.0, 6.0);
  CHECK_VFLOATREAL_EQUAL(expected_result, result);
  
  result = msub(v1,v2,v3);
  expected_result = vfloat4(42.0, 2.0, 2.0, 12.0);
  CHECK_VFLOATREAL_EQUAL(expected_result, result);
  
  result = nmadd(v1,v2,v3);
  expected_result = vfloat4(-42.0, -2.0, -2.0, -12.0);
  CHECK_VFLOATREAL_EQUAL(expected_result, result);

  result = nmsub(v1,v2,v3);
  expected_result = vfloat4(-30.0, 0.0, -6.0, -6.0);
  CHECK_VFLOATREAL_EQUAL(expected_result, result);

  vfloat4 v4 = -vfloat4(vals[0],vals[1], vals[2], vals[3]);
  result = max(v1,v4);
  expected_result = vfloat4(6.0, 1.0 , 2.0, 3.0);
  CHECK_VFLOATREAL_EQUAL(expected_result, result);

  result = min(v1,v4);
  expected_result = vfloat4(-6.0, -1.0 , -2.0, -3.0);
  CHECK_VFLOATREAL_EQUAL(expected_result, result);

  vfloat4 v5 = vfloat4(20.0, -12.0, 2.0, 100.0);
  
  result = max(v1,v4,v5);
  expected_result = vfloat4(20.0, 1.0, 2.0, 100.0);
  CHECK_VFLOATREAL_EQUAL(expected_result, result);

  result = min(v1,v4,v5);
  expected_result = vfloat4(-6.0, -12.0, -2.0, -3.0);
  CHECK_VFLOATREAL_EQUAL(expected_result, result);

  result = max(v1,v2,v4,v5);
  expected_result = vfloat4(20.0, 1.0, 2.0, 100.0);
  CHECK_VFLOATREAL_EQUAL(expected_result, result);

  result = min(v1,v2,v4,v5);
  expected_result = vfloat4(-6.0, -12.0, -2.0, -3.0);
  CHECK_VFLOATREAL_EQUAL(expected_result, result);

}

void test_vfloat4_comparators() {

  float vals[4] = {-6.0, 0.0, 2.0, -3.0};

  vfloat4 v1 = vfloat4(vals[0],vals[1], vals[2], vals[3]);
  
  vfloat4 v2 = -v1;

  vbool4 result = (v1 == v1);
  vbool4 expected_result = vbool4(true, true, true, true);
  CHECK_VBOOLREAL_EQUAL(expected_result, result);

  result = (v1 != v1);
  expected_result = vbool4(false, false, false, false);
  CHECK_VBOOLREAL_EQUAL(expected_result, result);

  result = (v1 < v2);
  expected_result = vbool4(true, false, false, true);
  CHECK_VBOOLREAL_EQUAL(expected_result, result);

  result = (v1 <= v2);
  expected_result = vbool4(true, true, false, true);
  CHECK_VBOOLREAL_EQUAL(expected_result, result);
  
  result = (v1 > v2);
  expected_result = vbool4(false, false, true, false);
  CHECK_VBOOLREAL_EQUAL(expected_result, result);

  result = (v1 >= v2);
  expected_result = vbool4(false, true, true, false);
  CHECK_VBOOLREAL_EQUAL(expected_result, result);

}

void test_vfloat4_load_store() {

  float vals1[4] = {-6.0, -1.0,  2.0, -3.0};
  float vals2[4] = { 6.0,  1.0, -2.0,  3.0};
  
  vfloat4 v1 = vfloat4(vals1[0],vals1[1], vals1[2], vals1[3]);
  vfloat4 v2 = vfloat4(vals2[0],vals2[1], vals2[2], vals2[3]);

  vfloat4 v3;

  v3 = vfloat4::load((void*)&v1);

  CHECK_VFLOATREAL_EQUAL(v1, v3);


  v3 = vfloat4::load((void*)&v2);

  CHECK_VFLOATREAL_EQUAL(v2, v3);

  vfloat4 v4;

  vfloat4::store((void*)&v4, v1);

  CHECK_VFLOATREAL_EQUAL(v1, v4);

}
