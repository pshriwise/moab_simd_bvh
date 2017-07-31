
#include "testutil.hpp"
#include "vbool.h"

void test_vbool4_constructors();
void test_vbool4_mask();

// void test_vbool4_operators();
// void test_vbool4_comparators();
// void test_vbool4_methods();

int main( int argc, char** argv ) {

  // test that values are correct after construction
  test_vbool4_constructors();
  // test vbool4 masking
  test_vbool4_mask();
  
  return 0;
}

void test_vbool4_constructors() {

  bool val = true;
  vbool4 v(val);

  CHECK_REAL_EQUAL(val, v[0], 0.0); 
  CHECK_REAL_EQUAL(val, v[1], 0.0);
  CHECK_REAL_EQUAL(val, v[2], 0.0);
  CHECK_REAL_EQUAL(val, v[3], 0.0);

  bool vals[4] = {true, false, true, false};
  
  v = vbool4(vals[0],vals[1], vals[2], vals[3]);
  
  CHECK_REAL_EQUAL(vals[0], v[0], 0.0); 
  CHECK_REAL_EQUAL(vals[1], v[1], 0.0);
  CHECK_REAL_EQUAL(vals[2], v[2], 0.0);
  CHECK_REAL_EQUAL(vals[3], v[3], 0.0);
  
  const void* const p = (void*)vals;
  v = vbool4::load(p);
  
  CHECK_REAL_EQUAL(vals[0], v[0], 0.0); 
  CHECK_REAL_EQUAL(vals[1], v[1], 0.0);
  CHECK_REAL_EQUAL(vals[2], v[2], 0.0);
  CHECK_REAL_EQUAL(vals[3], v[3], 0.0);

}

void test_vbool4_mask() {
  
  vbool4 v = vbool4(true, true, true, true);
  size_t result = movemask(v);
  size_t expected_result = 15;
  CHECK_EQUAL(expected_result,result);

  v = vbool4(false, true, true, true);
  result = movemask(v);
  expected_result = 14;
  CHECK_EQUAL(expected_result,result);

  v = vbool4(false, false, true, true);
  result = movemask(v);
  expected_result = 12;
  CHECK_EQUAL(expected_result,result);

  v = vbool4(false, false, false, true);
  result = movemask(v);
  expected_result = 8;
  CHECK_EQUAL(expected_result,result);

  v = vbool4(false,false,false,false);
  result = movemask(v);
  expected_result = 0;
  CHECK_EQUAL(expected_result,result);
  
  v = vbool4(true,false,true,false);
  result = movemask(v);
  expected_result = 5;
  CHECK_EQUAL(expected_result,result);
  
  v = vbool4(false,true,false,true);
  result = movemask(v);
  expected_result = 10;
  CHECK_EQUAL(expected_result,result);
}
