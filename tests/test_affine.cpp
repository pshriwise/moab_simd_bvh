

#include <stdio.h>
#include "space.h"
#include "testutil.hpp"

void constructor_tests();
void operator_tests();
void linear_algebra_tests();

#define CHECK_AFFINESPACE_EQUAL(A, B) check_affinespace_eq( (A), (B) )


bool check_affinespace_eq( const AffineSpaceV& A, const AffineSpaceV& B ) {
  
  // verify vector settings
  CHECK_VFLOATREAL_EQUAL(A.l.vx.x, B.l.vx.x);
  CHECK_VFLOATREAL_EQUAL(A.l.vx.y, B.l.vx.y);
  CHECK_VFLOATREAL_EQUAL(A.l.vx.z, B.l.vx.z);

  CHECK_VFLOATREAL_EQUAL(A.l.vy.x, B.l.vy.x);
  CHECK_VFLOATREAL_EQUAL(A.l.vy.y, B.l.vy.y);
  CHECK_VFLOATREAL_EQUAL(A.l.vy.z, B.l.vy.z);

  CHECK_VFLOATREAL_EQUAL(A.l.vz.x, B.l.vz.x);
  CHECK_VFLOATREAL_EQUAL(A.l.vz.y, B.l.vz.y);
  CHECK_VFLOATREAL_EQUAL(A.l.vz.z, B.l.vz.z);

  CHECK_VFLOATREAL_EQUAL(A.p.x, B.p.x);
  CHECK_VFLOATREAL_EQUAL(A.p.y, B.p.y);
  CHECK_VFLOATREAL_EQUAL(A.p.z, B.p.z);

}


int main(int argc, char** argv) {

  constructor_tests();
  operator_tests();
  
  return 0;
}

void constructor_tests() {

  // empty consttructor
  AffineSpaceV A;
  A = AffineSpaceV();
  
  // zero constructor
  A = AffineSpaceV(zero);
    
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
  A = AffineSpaceV(Vec3vfa(1.0, 0.0, 0.0),
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

  AffineSpaceV B = A;
  
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
  B = AffineSpaceV(A);
  
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

  A = AffineSpaceV(ls, v);

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
  AffineSpaceV A = AffineSpaceV(Vec3vfa(1.0, 0.0, 0.0),
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
  CHECK_EQUAL(A,A);

  // test mult operator
  float mult_val = 5.0f;
  AffineSpaceV B = mult_val * A;

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
  CHECK_VFLOATREAL_EQUAL(one_fifth, B.l.vx.x);
  CHECK_VFLOATREAL_EQUAL(zero_vec,  B.l.vx.y);
  CHECK_VFLOATREAL_EQUAL(zero_vec,  B.l.vx.z);

  CHECK_VFLOATREAL_EQUAL(zero_vec,  B.l.vy.x);
  CHECK_VFLOATREAL_EQUAL(one_fifth, B.l.vy.y);
  CHECK_VFLOATREAL_EQUAL(zero_vec,  B.l.vy.z);

  CHECK_VFLOATREAL_EQUAL(zero_vec,  B.l.vz.x);
  CHECK_VFLOATREAL_EQUAL(zero_vec,  B.l.vz.y);
  CHECK_VFLOATREAL_EQUAL(one_fifth, B.l.vz.z);

  CHECK_VFLOATREAL_EQUAL(one_fifth, B.p.x);
  CHECK_VFLOATREAL_EQUAL(one_fifth, B.p.y);
  CHECK_VFLOATREAL_EQUAL(one_fifth, B.p.z);

  // multiply by this value
  B *= mult_val;

  // should be the same as A now
  CHECK_EQUAL(A,B);

  // divide using unary operator
  B /= mult_val;

  // verify vector settings
  CHECK_VFLOATREAL_EQUAL(one_fifth, B.l.vx.x);
  CHECK_VFLOATREAL_EQUAL(zero_vec,  B.l.vx.y);
  CHECK_VFLOATREAL_EQUAL(zero_vec,  B.l.vx.z);

  CHECK_VFLOATREAL_EQUAL(zero_vec,  B.l.vy.x);
  CHECK_VFLOATREAL_EQUAL(one_fifth, B.l.vy.y);
  CHECK_VFLOATREAL_EQUAL(zero_vec,  B.l.vy.z);

  CHECK_VFLOATREAL_EQUAL(zero_vec,  B.l.vz.x);
  CHECK_VFLOATREAL_EQUAL(zero_vec,  B.l.vz.y);
  CHECK_VFLOATREAL_EQUAL(one_fifth, B.l.vz.z);

  CHECK_VFLOATREAL_EQUAL(one_fifth, B.p.x);
  CHECK_VFLOATREAL_EQUAL(one_fifth, B.p.y);
  CHECK_VFLOATREAL_EQUAL(one_fifth, B.p.z);


  AffineSpaceV C;

  // multiply by identity
  C = A * B;

  AffineSpaceV expected_result(B);
  expected_result.p = A.p + B.p;
  CHECK_EQUAL(expected_result, C);

  // multiply by self
  C = B * B;

  // verify vector settings
  CHECK_VFLOATREAL_EQUAL(one_fifth*one_fifth, C.l.vx.x);
  CHECK_VFLOATREAL_EQUAL(zero_vec,            C.l.vx.y);
  CHECK_VFLOATREAL_EQUAL(zero_vec,            C.l.vx.z);

  CHECK_VFLOATREAL_EQUAL(zero_vec,            C.l.vy.x);
  CHECK_VFLOATREAL_EQUAL(one_fifth*one_fifth, C.l.vy.y);
  CHECK_VFLOATREAL_EQUAL(zero_vec,            C.l.vy.z);

  CHECK_VFLOATREAL_EQUAL(zero_vec,            C.l.vz.x);
  CHECK_VFLOATREAL_EQUAL(zero_vec,            C.l.vz.y);
  CHECK_VFLOATREAL_EQUAL(one_fifth*one_fifth, C.l.vz.z);

  CHECK_VFLOATREAL_EQUAL(one_fifth+ one_fifth*one_fifth, C.p.x);
  CHECK_VFLOATREAL_EQUAL(one_fifth+ one_fifth*one_fifth, C.p.y);
  CHECK_VFLOATREAL_EQUAL(one_fifth+ one_fifth*one_fifth, C.p.z);

  // addition op
  C = A + B;
  // verify vector settings
  CHECK_VFLOATREAL_EQUAL(one_fifth+pos_one_vec, C.l.vx.x);
  CHECK_VFLOATREAL_EQUAL(zero_vec,              C.l.vx.y);
  CHECK_VFLOATREAL_EQUAL(zero_vec,              C.l.vx.z);

  CHECK_VFLOATREAL_EQUAL(zero_vec,              C.l.vy.x);
  CHECK_VFLOATREAL_EQUAL(one_fifth+pos_one_vec, C.l.vy.y);
  CHECK_VFLOATREAL_EQUAL(zero_vec,              C.l.vy.z);

  CHECK_VFLOATREAL_EQUAL(zero_vec,              C.l.vz.x);
  CHECK_VFLOATREAL_EQUAL(zero_vec,              C.l.vz.y);
  CHECK_VFLOATREAL_EQUAL(one_fifth+pos_one_vec, C.l.vz.z);

  CHECK_VFLOATREAL_EQUAL(one_fifth+pos_one_vec, C.p.x);
  CHECK_VFLOATREAL_EQUAL(one_fifth+pos_one_vec, C.p.y);
  CHECK_VFLOATREAL_EQUAL(one_fifth+pos_one_vec, C.p.z);

  C = B - A;
  
  // verify vector settings
  CHECK_VFLOATREAL_EQUAL(one_fifth-pos_one_vec, C.l.vx.x);
  CHECK_VFLOATREAL_EQUAL(zero_vec,              C.l.vx.y);
  CHECK_VFLOATREAL_EQUAL(zero_vec,              C.l.vx.z);

  CHECK_VFLOATREAL_EQUAL(zero_vec,              C.l.vy.x);
  CHECK_VFLOATREAL_EQUAL(one_fifth-pos_one_vec, C.l.vy.y);
  CHECK_VFLOATREAL_EQUAL(zero_vec,              C.l.vy.z);

  CHECK_VFLOATREAL_EQUAL(zero_vec,              C.l.vz.x);
  CHECK_VFLOATREAL_EQUAL(zero_vec,              C.l.vz.y);
  CHECK_VFLOATREAL_EQUAL(one_fifth-pos_one_vec, C.l.vz.z);

  CHECK_VFLOATREAL_EQUAL(one_fifth-pos_one_vec, C.p.x);
  CHECK_VFLOATREAL_EQUAL(one_fifth-pos_one_vec, C.p.y);
  CHECK_VFLOATREAL_EQUAL(one_fifth-pos_one_vec, C.p.z);

  // reciprocal operator
  C = rcp(A);

  LinSpaceV lv = A.l;

  // C's linear space should match the inverse of A's
  CHECK_EQUAL(lv.inverse(), C.l);
  // because A's linear space is the identity, it shouldn't have changed
  CHECK_EQUAL(lv, C.l);

  // the new vector should have flipped signs
  CHECK_EQUAL(-A.p, C.p);

  // reciprocal operator
  C = rcp(B);

  lv = B.l;

  CHECK_EQUAL(lv.inverse(), C.l);

  CHECK_EQUAL(-lv.inverse()*B.p, C.p);  
}
 

