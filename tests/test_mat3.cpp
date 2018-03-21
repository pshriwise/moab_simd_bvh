

#include <iostream>
#include "testutil.hpp"
#include "mat3.h"

int main(int argc, char** argv) {

  //Create a matrix
  Matrix3 mat;

  mat(0) = 2;
  mat(1) = -1;
  mat(2) = 0;
  mat(3) = -1;
  mat(4) = 2;
  mat(5) = -1;
  mat(6) = 0;
  mat(7) = -1;
  mat(8) = 2;

  //now do the Eigen Decomposition of this Matrix

  float lambda[3]; 
  Vec3fa vectors[3];
  Matrix::EigenDecomp(mat, lambda, vectors);

  for (int i = 0; i < 3; i++) vectors[i].normalize();

  //Hardcoded check values for eigen value results
  float lambda_check[3];
  lambda_check[0] = 3.41421; lambda_check[1] = 2.0; lambda_check[2] = 0.585786; 
  Vec3fa vec0_check(0.5, -0.707107, 0.5);
  Vec3fa vec1_check(0.707107, 3.37748e-17, -0.707107);
  Vec3fa vec2_check(0.5, 0.707107, 0.5);  
  //now verfy that the returns Eigenvalues and Eigenvectors are correct (within some tolerance)
  float tol = 1e-04;

  CHECK_REAL_EQUAL(lambda[0], lambda_check[0], tol);
  CHECK_REAL_EQUAL(lambda[1], lambda_check[1], tol);
  CHECK_REAL_EQUAL(lambda[2], lambda_check[2], tol);

  CHECK_VECREAL_EQUAL_TOL(vectors[0], vec0_check, tol);
  
  return 0;
}
