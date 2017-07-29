

#include "vfloat.h"
#include "testutil.hpp"

int main( int argc, char** argv ) {

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

  const void* const p = (void*)vals;
  v = vfloat4::load(p);
  
  CHECK_REAL_EQUAL(vals[0], v[0], 0.0); 
  CHECK_REAL_EQUAL(vals[1], v[1], 0.0);
  CHECK_REAL_EQUAL(vals[2], v[2], 0.0);
  CHECK_REAL_EQUAL(vals[3], v[3], 0.0);
  
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
  
  return 0;
}
