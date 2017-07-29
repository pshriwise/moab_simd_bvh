#include <stdio.h>

#include "Ray.h"
#include "testutil.hpp"


void test_constructors();

int main ( int agc, char** argv ) {
  // test ray constructors
  test_constructors();
  return 0;
}

void test_constructors() {

  Vec3f origin(6.0, 6.0, 6.0);
  Vec3f direction(1.0, 0.0, 0.0);
  
  // test minimal  constructor
  Ray r = Ray(origin, direction);
  CHECK_VECREAL_EQUAL(origin, r.org);
  CHECK_VECREAL_EQUAL(direction, r.dir);
  CHECK_REAL_EQUAL(zero, r.tnear, 0.0);
  CHECK_INF(r.tfar);
  CHECK_EQUAL(-1, r.mask);
  
  // test constructor with tnear
  float tn = 1.0;
  r = Ray(origin, direction, tn);
  CHECK_VECREAL_EQUAL(origin, r.org);
  CHECK_VECREAL_EQUAL(direction, r.dir);
  CHECK_REAL_EQUAL(tn, r.tnear, 0.0);
  CHECK_INF(r.tfar);
  CHECK_EQUAL(-1, r.mask);

  //test constructor with tfar
  float tf = 500.0;
  r = Ray(origin, direction, tn, tf);
  CHECK_VECREAL_EQUAL(origin, r.org);
  CHECK_VECREAL_EQUAL(direction, r.dir);
  CHECK_REAL_EQUAL(tn, r.tnear, 0.0);
  CHECK_REAL_EQUAL(tf, r.tfar, 0.0);
  CHECK_EQUAL(-1, r.mask);


  //test constructor with mask
  int msk = 6;
  r = Ray(origin, direction, tn, tf, msk);
  CHECK_VECREAL_EQUAL(origin, r.org);
  CHECK_VECREAL_EQUAL(direction, r.dir);
  CHECK_REAL_EQUAL(tn, r.tnear, 0.0);
  CHECK_REAL_EQUAL(tf, r.tfar, 0.0);
  CHECK_EQUAL(msk, r.mask);
    
  // test empty constructor
  r = Ray();

  TravRay tr(origin, direction);
  CHECK_VECREAL_EQUAL(origin, tr.org_xyz);
  CHECK_VECREAL_EQUAL(direction, tr.dir_xyz);
  CHECK_VECREAL_EQUAL(origin, tr.org);
  CHECK_VECREAL_EQUAL(direction, tr.dir);
  CHECK_EQUAL(0*sizeof(vfloat4), tr.nearX);
  CHECK_EQUAL(2*sizeof(vfloat4), tr.nearY);
  CHECK_EQUAL(4*sizeof(vfloat4), tr.nearZ);
  CHECK_EQUAL(16,(int)tr.farX);
  CHECK_EQUAL(48,(int)tr.farY);
  CHECK_EQUAL(80,(int)tr.farZ);

  tr = TravRay(origin, -direction);
  CHECK_VECREAL_EQUAL(origin, tr.org_xyz);
  CHECK_VECREAL_EQUAL(-direction, tr.dir_xyz);
  CHECK_VECREAL_EQUAL(origin, tr.org);
  CHECK_VECREAL_EQUAL(-direction, tr.dir);
  CHECK_EQUAL(1*sizeof(vfloat4), tr.nearX);
  CHECK_EQUAL(2*sizeof(vfloat4), tr.nearY);
  CHECK_EQUAL(4*sizeof(vfloat4), tr.nearZ);
  CHECK_EQUAL(0,(int)tr.farX);
  CHECK_EQUAL(48,(int)tr.farY);
  CHECK_EQUAL(80,(int)tr.farZ);

}


