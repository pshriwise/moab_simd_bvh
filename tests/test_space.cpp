
#include <stdio.h>
#include "space.h"
#include "testutil.hpp"

#define EPS 1e-06

void constructor_tests();
void operator_tests();
void linear_algebra_tests();

int main(int argc, char** argv) {

  constructor_tests();
  operator_tests();
  linear_algebra_tests();
  
  return 0;
}

void constructor_tests() {

  // column vector constructor
  Vec3fa ax0(0.0f, 1.0f, 0.0f);
  Vec3fa ax1(1.0f, 0.0f, 0.0f);
  Vec3fa ax2(0.0f, 0.0f, 1.0f);

  // vector constructor
  LinSpace ls(ax0, ax1, ax2);

  CHECK_VECREAL_EQUAL(ax0, ls.row0());
  CHECK_VECREAL_EQUAL(ax1, ls.row1());
  CHECK_VECREAL_EQUAL(ax2, ls.row2());

  // explicit value constructor
  LinSpace ls1(0.0f, 1.0f, 0.0f,
	       1.0f, 0.0f, 0.0f,
	       0.0f, 0.0f, 1.0f);


  CHECK_EQUAL(ls, ls1);

  // copy constructor
  LinSpace ls2 = ls;

  CHECK_EQUAL(ls, ls2);
}

void operator_tests() {

  LinSpace ls( 2, -1,  0,
	       -1,  2, -1,
	       0, -1,  2);

  // check division opterator
  LinSpace ls_divided(    1, -0.5,    0,
		       -0.5,    1, -0.5,
		          0, -0.5,    1);

  CHECK_EQUAL(ls_divided, ls / 2.0);

  // check negative operator
  LinSpace ls_neg( -2,  1,  0,
	            1, -2,  1,
                    0,  1, -2);

  CHECK_EQUAL(ls_neg, -ls);

  // check positive operator
  CHECK_EQUAL(ls, +ls);

  // check multiply by value operator
  float multiply_val = 3.0;

  LinSpace expected_mult_val_result( 6, -3, 0,
				    -3, 6, -3,
  				     0, -3, 6);
  
  CHECK_EQUAL(expected_mult_val_result, multiply_val * ls);

  // in-place multiplication check
  ls *= multiply_val;
  
  CHECK_EQUAL(expected_mult_val_result, ls);

  // in place division operator
  ls /= multiply_val;


  // multiply by vector operator
  Vec3fa multiply_vec(2.0, 3.0, 4.0);

  Vec3fa expected_mult_vec_result(1.0, 0.0, 5.0);

  CHECK_VECREAL_EQUAL(expected_mult_vec_result, ls * multiply_vec);

  // multiply by linear space operator
  LinSpace other_ls(1, 2, 3,
		    4, 5, 6,
		    7, 8, 9);

  LinSpace expected_mult_ls_result(0, 0, 4,
				   3, 0, 7,
				   6, 0, 10);
  
  CHECK_EQUAL(expected_mult_ls_result, other_ls * ls);
  
}

void linear_algebra_tests() {

  // test transpose
  LinSpace ls(3, 2, 3,
	      4, 5, 6,
	      7, 8, 9);

  Vec3fa row0( 3, 2, 3);
  Vec3fa row1( 4, 5, 6);
  Vec3fa row2( 7, 8, 9);

  CHECK_VECREAL_EQUAL(row0, ls.row0());
  CHECK_VECREAL_EQUAL(row1, ls.row1());
  CHECK_VECREAL_EQUAL(row2, ls.row2());

  LinSpace ls_transpose(row0, row1, row2);

  CHECK_EQUAL(ls_transpose, ls.transpose());

  // test determinant
  float expected_determinant = -6.0;

  CHECK_EQUAL(expected_determinant, ls.det());

  // test inverse
  LinSpace expected_inverse(  3, -6, 3,
			     -6, -6, 6,
			      3, 10, -7);
  expected_inverse *= (1.0f/6.0f);

  CHECK_VECREAL_EQUAL_TOL(expected_inverse.row0(), ls.inverse().row0(), EPS);
  CHECK_VECREAL_EQUAL_TOL(expected_inverse.row1(), ls.inverse().row1(), EPS);
  CHECK_VECREAL_EQUAL_TOL(expected_inverse.row2(), ls.inverse().row2(), EPS);

  // test adjoint
  LinSpace expected_adjoint( -3,   6, -3,
			      6,   6, -6,
			     -3, -10,  7);


  CHECK_VECREAL_EQUAL_TOL(expected_adjoint.row0(), ls.adjoint().row0(), EPS);
  CHECK_VECREAL_EQUAL_TOL(expected_adjoint.row1(), ls.adjoint().row1(), EPS);
  CHECK_VECREAL_EQUAL_TOL(expected_adjoint.row2(), ls.adjoint().row2(), EPS);

}

