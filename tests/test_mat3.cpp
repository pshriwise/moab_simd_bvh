

#include <iostream>
#include "testutil.hpp"
#include "mat3.h"



#define ACOS(x) acos(std::min(std::max(x,-1.0f),1.0f))

float find_angle(const Vec3fa& A, const Vec3fa& B)
{
  const float lA=A.length(), lB=B.length();
  assert(lA > 0.0);
  assert(lB > 0.0);
  const float dPI = 3.14159265;
  const float dot=(A[0]*B[0]+A[1]*B[1]+A[2]*B[2]);
  return ACOS( dot / ( lA * lB ) ) * 180.0 / dPI;
}

#define CHECK_EIGVECREAL_EQUAL( EXP, ACT, EPS ) check_equal_eigvect( (EXP), (ACT), (EPS), #EXP, #ACT, __LINE__, __FILE__ ) 
void check_equal_eigvect( const Vec3fa& A,
                        const Vec3fa& B, float eps,
                        const char* sA, const char* sB, 
                        int line, const char* file )
{
  check_equal( A.length(), B.length(), eps, sA, sB, line, file);

  float angle = find_angle(A, B);

  if (  (fabs(A[0] - B[0]) <= eps || fabs(A[0] + B[0]) <= eps) && 
        (fabs(A[1] - B[1]) <= eps || fabs(A[1] + B[1]) <= eps) &&
        (fabs(A[2] - B[2]) <= eps || fabs(A[2] + B[2]) <= eps) && 
        (angle <= eps || fabs(angle - 180.0) <= eps) )
    return;
  
  std::cout << "Equality Test Failed: " << sA << " == " << sB << std::endl;
  std::cout << "  at line " << line << " of '" << file << "'" << std::endl;
   
  std::cout << "  Expected: ";
  std::cout << A << std::endl;
  
  std::cout << "  Actual:   ";
  std::cout << B << std::endl;

  std::cout << "Angle between vectors := " << angle << std::endl;
  
  flag_error(); 
}

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
  Vec3fa vec1_check(-0.707107, 3.37748e-17, 0.707107);
  Vec3fa vec2_check(0.5, 0.707107, 0.5);  
  //now verfy that the returns Eigenvalues and Eigenvectors are correct (within some tolerance)
  float tol = 1e-04;

  // make sure the expected lambda values match
  CHECK_REAL_EQUAL(lambda_check[0], lambda[0], tol);
  CHECK_REAL_EQUAL(lambda_check[1], lambda[1], tol);
  CHECK_REAL_EQUAL(lambda_check[2], lambda[2], tol);

  // make sure the expected eigenvectors match
  CHECK_EIGVECREAL_EQUAL(vec0_check, vectors[0], tol);
  CHECK_EIGVECREAL_EQUAL(vec1_check, vectors[1], tol);
  CHECK_EIGVECREAL_EQUAL(vec2_check, vectors[2], tol);


  // check that the result is valid (AM - kM = 0)
  for(int i = 0; i < 3; i++) {
    Vec3fa check_vec = Matrix::matrix_vector(mat, vectors[i]) - lambda[i] * vectors[i];
    CHECK_REAL_EQUAL(check_vec.length(), 0.0f, tol);
  }
  
  // check for real, symmetric matrix of returned eigenvectors
  CHECK_REAL_EQUAL( dot(vectors[0],vectors[1]), 0.0f, tol);
  CHECK_REAL_EQUAL( dot(vectors[0],vectors[2]), 0.0f, tol);
  
  return 0;
}
