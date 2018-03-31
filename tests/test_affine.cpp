

#include <stdio.h>
#include "space.h"
#include "testutil.hpp"

#define EPS 0.0f

void constructor_tests();
void operator_tests();
void linear_algebra_tests();


int main(int argc, char** argv) {

  constructor_tests();
  operator_tests();
  
  return 0;
}

void constructor_tests() {

  // empty consttructor
  AffineSpace A;
  A = AffineSpace();
  
  // zero constructor
  A = AffineSpace(zero);
    
  // expected vector
  vfloat4 zero_vec(zero);
  
  // all of the things should be zero at this point
  CHECK_VFLOATREAL_EQUAL(zero_vec, A.l.vx.x);
  CHECK_VFLOATREAL_EQUAL(zero_vec, A.l.vx.y);
  CHECK_VFLOATREAL_EQUAL(zero_vec, A.l.vx.z);

  CHECK_VFLOATREAL_EQUAL(zero_vec, A.l.vy.x);
  CHECK_VFLOATREAL_EQUAL(zero_vec, A.l.vy.y);
  CHECK_VFLOATREAL_EQUAL(zero_vec, A.l.vy.z);

  CHECK_VFLOATREAL_EQUAL(zero_vec, A.l.vz.x);
  CHECK_VFLOATREAL_EQUAL(zero_vec, A.l.vz.y);
  CHECK_VFLOATREAL_EQUAL(zero_vec, A.l.vz.z);

  CHECK_VFLOATREAL_EQUAL(zero_vec, A.p.x);
  CHECK_VFLOATREAL_EQUAL(zero_vec, A.p.y);
  CHECK_VFLOATREAL_EQUAL(zero_vec, A.p.z);

  // vector constructor
  A = AffineSpace(Vec3vfa(1.0, 0.0, 0.0),
		  Vec3vfa(0.0, 1.0, 0.0),
		  Vec3vfa(0.0, 0.0, 1.0),
		  Vec3fa(1.0, 1.0, 1.0));

  vfloat4 one_vec(1.0f);

  // verify vector settings
  CHECK_VFLOATREAL_EQUAL(one_vec,  A.l.vx.x);
  CHECK_VFLOATREAL_EQUAL(zero_vec, A.l.vx.y);
  CHECK_VFLOATREAL_EQUAL(zero_vec, A.l.vx.z);

  CHECK_VFLOATREAL_EQUAL(zero_vec, A.l.vy.x);
  CHECK_VFLOATREAL_EQUAL(one_vec,  A.l.vy.y);
  CHECK_VFLOATREAL_EQUAL(zero_vec, A.l.vy.z);

  CHECK_VFLOATREAL_EQUAL(zero_vec, A.l.vz.x);
  CHECK_VFLOATREAL_EQUAL(zero_vec, A.l.vz.y);
  CHECK_VFLOATREAL_EQUAL(one_vec,  A.l.vz.z);

  CHECK_VFLOATREAL_EQUAL(one_vec, A.p.x);
  CHECK_VFLOATREAL_EQUAL(one_vec, A.p.y);
  CHECK_VFLOATREAL_EQUAL(one_vec, A.p.z);

  AffineSpace B = A;
  
  // verify vector settings
  CHECK_VFLOATREAL_EQUAL(one_vec,  B.l.vx.x);
  CHECK_VFLOATREAL_EQUAL(zero_vec, B.l.vx.y);
  CHECK_VFLOATREAL_EQUAL(zero_vec, B.l.vx.z);

  CHECK_VFLOATREAL_EQUAL(zero_vec, B.l.vy.x);
  CHECK_VFLOATREAL_EQUAL(one_vec,  B.l.vy.y);
  CHECK_VFLOATREAL_EQUAL(zero_vec, B.l.vy.z);

  CHECK_VFLOATREAL_EQUAL(zero_vec, B.l.vz.x);
  CHECK_VFLOATREAL_EQUAL(zero_vec, B.l.vz.y);
  CHECK_VFLOATREAL_EQUAL(one_vec,  B.l.vz.z);

  CHECK_VFLOATREAL_EQUAL(one_vec, B.p.x);
  CHECK_VFLOATREAL_EQUAL(one_vec, B.p.y);
  CHECK_VFLOATREAL_EQUAL(one_vec, B.p.z);

  // another copy constructor
  B = AffineSpace(A);
  
  // verify vector settings
  CHECK_VFLOATREAL_EQUAL(one_vec,  B.l.vx.x);
  CHECK_VFLOATREAL_EQUAL(zero_vec, B.l.vx.y);
  CHECK_VFLOATREAL_EQUAL(zero_vec, B.l.vx.z);

  CHECK_VFLOATREAL_EQUAL(zero_vec, B.l.vy.x);
  CHECK_VFLOATREAL_EQUAL(one_vec,  B.l.vy.y);
  CHECK_VFLOATREAL_EQUAL(zero_vec, B.l.vy.z);

  CHECK_VFLOATREAL_EQUAL(zero_vec, B.l.vz.x);
  CHECK_VFLOATREAL_EQUAL(zero_vec, B.l.vz.y);
  CHECK_VFLOATREAL_EQUAL(one_vec,  B.l.vz.z);

  CHECK_VFLOATREAL_EQUAL(one_vec, B.p.x);
  CHECK_VFLOATREAL_EQUAL(one_vec, B.p.y);
  CHECK_VFLOATREAL_EQUAL(one_vec, B.p.z);


  // linear space and scale vector constructor
  LinSpaceV ls( 2, -1,  0,
	       -1,  2, -1,
	        0, -1,  2);

  Vec3vfa v(10.0, 10.0, 10.0);

  A = AffineSpace(ls, v);

  vfloat4 pos_two_vec( 2.0);
  vfloat4 neg_one_vec(-1.0);
  vfloat4 pos_ten_vec(10.0);

  // verify vector settings
  CHECK_VFLOATREAL_EQUAL(pos_two_vec, A.l.vx.x);
  CHECK_VFLOATREAL_EQUAL(neg_one_vec, A.l.vx.y);
  CHECK_VFLOATREAL_EQUAL(   zero_vec, A.l.vx.z);

  CHECK_VFLOATREAL_EQUAL(neg_one_vec, A.l.vy.x);
  CHECK_VFLOATREAL_EQUAL(pos_two_vec, A.l.vy.y);
  CHECK_VFLOATREAL_EQUAL(neg_one_vec, A.l.vy.z);

  CHECK_VFLOATREAL_EQUAL(   zero_vec, A.l.vz.x);
  CHECK_VFLOATREAL_EQUAL(neg_one_vec, A.l.vz.y);
  CHECK_VFLOATREAL_EQUAL(pos_two_vec, A.l.vz.z);

  CHECK_VFLOATREAL_EQUAL(pos_ten_vec, A.p.x);
  CHECK_VFLOATREAL_EQUAL(pos_ten_vec, A.p.y);
  CHECK_VFLOATREAL_EQUAL(pos_ten_vec, A.p.z);

}
  

void operator_tests() {

  // vector constructor
  AffineSpace A = AffineSpace(Vec3vfa(1.0, 0.0, 0.0),
			      Vec3vfa(0.0, 1.0, 0.0),
			      Vec3vfa(0.0, 0.0, 1.0),
			      Vec3fa(1.0, 1.0, 1.0));

  // expected vector
  vfloat4 zero_vec(zero);
  vfloat4 pos_one_vec(1.0f);
  
  // verify vector settings
  CHECK_VFLOATREAL_EQUAL(pos_one_vec, A.l.vx.x);
  CHECK_VFLOATREAL_EQUAL(zero_vec,    A.l.vx.y);
  CHECK_VFLOATREAL_EQUAL(zero_vec,    A.l.vx.z);

  CHECK_VFLOATREAL_EQUAL(zero_vec,    A.l.vy.x);
  CHECK_VFLOATREAL_EQUAL(pos_one_vec, A.l.vy.y);
  CHECK_VFLOATREAL_EQUAL(zero_vec,    A.l.vy.z);

  CHECK_VFLOATREAL_EQUAL(zero_vec,    A.l.vz.x);
  CHECK_VFLOATREAL_EQUAL(zero_vec,    A.l.vz.y);
  CHECK_VFLOATREAL_EQUAL(pos_one_vec, A.l.vz.z);

  CHECK_VFLOATREAL_EQUAL(pos_one_vec, A.p.x);
  CHECK_VFLOATREAL_EQUAL(pos_one_vec, A.p.y);
  CHECK_VFLOATREAL_EQUAL(pos_one_vec, A.p.z);

  // eq operator check
  CHECK( A == A );

  // test mult operator
  float mult_val = 5.0f;
  AffineSpace B = mult_val * A;

  vfloat4 pos_five_vec(5.0f);

  // verify vector settings
  CHECK_VFLOATREAL_EQUAL(pos_five_vec, B.l.vx.x);
  CHECK_VFLOATREAL_EQUAL(zero_vec,    B.l.vx.y);
  CHECK_VFLOATREAL_EQUAL(zero_vec,    B.l.vx.z);

  CHECK_VFLOATREAL_EQUAL(zero_vec,    B.l.vy.x);
  CHECK_VFLOATREAL_EQUAL(pos_five_vec, B.l.vy.y);
  CHECK_VFLOATREAL_EQUAL(zero_vec,    B.l.vy.z);

  CHECK_VFLOATREAL_EQUAL(zero_vec,    B.l.vz.x);
  CHECK_VFLOATREAL_EQUAL(zero_vec,    B.l.vz.y);
  CHECK_VFLOATREAL_EQUAL(pos_five_vec, B.l.vz.z);

  CHECK_VFLOATREAL_EQUAL(pos_five_vec, B.p.x);
  CHECK_VFLOATREAL_EQUAL(pos_five_vec, B.p.y);
  CHECK_VFLOATREAL_EQUAL(pos_five_vec, B.p.z);

  // neq operator check
  CHECK ( A != B );
  
  // negative operator
  B = -B;
  
  vfloat4 neg_five_vec(-5.0f);
   
  // verify vector settings
  CHECK_VFLOATREAL_EQUAL(neg_five_vec, B.l.vx.x);
  CHECK_VFLOATREAL_EQUAL(zero_vec,     B.l.vx.y);
  CHECK_VFLOATREAL_EQUAL(zero_vec,     B.l.vx.z);

  CHECK_VFLOATREAL_EQUAL(zero_vec,     B.l.vy.x);
  CHECK_VFLOATREAL_EQUAL(neg_five_vec, B.l.vy.y);
  CHECK_VFLOATREAL_EQUAL(zero_vec,     B.l.vy.z);

  CHECK_VFLOATREAL_EQUAL(zero_vec,     B.l.vz.x);
  CHECK_VFLOATREAL_EQUAL(zero_vec ,    B.l.vz.y);
  CHECK_VFLOATREAL_EQUAL(neg_five_vec, B.l.vz.z);

  CHECK_VFLOATREAL_EQUAL(neg_five_vec, B.p.x);
  CHECK_VFLOATREAL_EQUAL(neg_five_vec, B.p.y);
  CHECK_VFLOATREAL_EQUAL(neg_five_vec, B.p.z);

  // check division operator

  B = A / mult_val;

  vfloat4 one_fifth(1.0/mult_val);

  // verify vector settings
  CHECK_VFLOATREAL_EQUAL_TOL(one_fifth, B.l.vx.x, EPS);
  CHECK_VFLOATREAL_EQUAL_TOL(zero_vec,  B.l.vx.y, EPS);
  CHECK_VFLOATREAL_EQUAL_TOL(zero_vec,  B.l.vx.z, EPS);

  CHECK_VFLOATREAL_EQUAL_TOL(zero_vec,  B.l.vy.x, EPS);
  CHECK_VFLOATREAL_EQUAL_TOL(one_fifth, B.l.vy.y, EPS);
  CHECK_VFLOATREAL_EQUAL_TOL(zero_vec,  B.l.vy.z, EPS);

  CHECK_VFLOATREAL_EQUAL_TOL(zero_vec,  B.l.vz.x, EPS);
  CHECK_VFLOATREAL_EQUAL_TOL(zero_vec,  B.l.vz.y, EPS);
  CHECK_VFLOATREAL_EQUAL_TOL(one_fifth, B.l.vz.z, EPS);

  CHECK_VFLOATREAL_EQUAL_TOL(one_fifth, B.p.x,    EPS);
  CHECK_VFLOATREAL_EQUAL_TOL(one_fifth, B.p.y,    EPS);
  CHECK_VFLOATREAL_EQUAL_TOL(one_fifth, B.p.z,    EPS);

  
  
}
