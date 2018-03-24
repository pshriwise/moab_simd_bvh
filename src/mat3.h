//// MODIFIED MOAB Version STARTING AT HASH ////
/// 19c46ab8b400a68e4645b3f501049c7669efd0e2 ///
//// OR VERSION ///
/// Version4.8.2 ///
/*
 * MOAB, a Mesh-Oriented datABase, is a software component for creating,
 * storing and accessing finite element mesh data.
 * 
 * Copyright 2004 Sandia Corporation.  Under the terms of Contract
 * DE-AC04-94AL85000 with Sandia Corporation, the U.S. Government
 * retains certain rights in this software.
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

/**\file Matrix3.hpp
 *\author Jason Kraftcheck (kraftche@cae.wisc.edu)
 *\date 2006-07-18
 *\date 2012-08-2 Updated by rhl to be more generic. less code that does more!
 * TODO: Remove all 'inline' keywords as it is only a suggestion to the compiler
 * anyways, and it will ignore it or add it when it thinks its necessary.
 */

#pragma once

#include <iostream>

#include <iosfwd>
#include <limits>
#include <float.h>
#include <assert.h>
#include <vector>
#include "Vec3fa.h"


#ifdef _MSC_VER
# define finite _finite
#endif

namespace Matrix{
	template< typename Matrix>
	Matrix inverse( const Matrix & d, const float i){
		Matrix m( d);
		 m( 0) = i * (d(4) * d(8) - d(5) * d(7));
	         m( 1) = i * (d(2) * d(7) - d(8) * d(1));
	         m( 2) = i * (d(1) * d(5) - d(4) * d(2));
	         m( 3) = i * (d(5) * d(6) - d(8) * d(3));
	         m( 4) = i * (d(0) * d(8) - d(6) * d(2));
	         m( 5) = i * (d(2) * d(3) - d(5) * d(0));
	         m( 6) = i * (d(3) * d(7) - d(6) * d(4));
	         m( 7) = i * (d(1) * d(6) - d(7) * d(0));
	         m( 8) = i * (d(0) * d(4) - d(3) * d(1));
		return m;
	}

	template< typename Matrix>
	__forceinline bool positive_definite( const Matrix & d, 
				       float& det ){
	        float subdet6 = d(1)*d(5)-d(2)*d(4);
	        float subdet7 = d(2)*d(3)-d(0)*d(5);
	        float subdet8 = d(0)*d(4)-d(1)*d(3);
	        det = d(6)*subdet6 + d(7)*subdet7 + d(8)*subdet8;
	        return d(0) > 0 && subdet8 > 0 && det > 0;
	}
	template< typename Matrix>
	__forceinline Matrix transpose( const Matrix & d){
	      return Matrix( d(0), d(3), d(6),
	                     d(1), d(4), d(7),
	                     d(2), d(5), d(8) );
	}
	template< typename Matrix>
	__forceinline Matrix mmult3( const Matrix& a, const Matrix& b ) {
	  return Matrix( a(0,0) * b(0,0) + a(0,1) * b(1,0) + a(0,2) * b(2,0),
	                 a(0,0) * b(0,1) + a(0,1) * b(1,1) + a(0,2) * b(2,1),
	                 a(0,0) * b(0,2) + a(0,1) * b(1,2) + a(0,2) * b(2,2),
	                 a(1,0) * b(0,0) + a(1,1) * b(1,0) + a(1,2) * b(2,0),
	                 a(1,0) * b(0,1) + a(1,1) * b(1,1) + a(1,2) * b(2,1),
	                 a(1,0) * b(0,2) + a(1,1) * b(1,2) + a(1,2) * b(2,2),
	                 a(2,0) * b(0,0) + a(2,1) * b(1,0) + a(2,2) * b(2,0),
	                 a(2,0) * b(0,1) + a(2,1) * b(1,1) + a(2,2) * b(2,1),
	                 a(2,0) * b(0,2) + a(2,1) * b(1,2) + a(2,2) * b(2,2) );
	}

	template< typename Vector, typename Matrix>
	__forceinline Matrix outer_product( const Vector & u,
	                              const Vector & v,
				      Matrix & m ) {
	  	m = Matrix( u[0] * v[0], u[0] * v[1], u[0] * v[2],
	                    u[1] * v[0], u[1] * v[1], u[1] * v[2],
	                    u[2] * v[0], u[2] * v[1], u[2] * v[2] );
		return m;
	}
	template< typename Matrix>
	__forceinline float determinant3( const Matrix & d){
		return (d(0) * d(4) * d(8) 
		     + d(1) * d(5) * d(6)
		     + d(2) * d(3) * d(7)
		     - d(0) * d(5) * d(7)
		     - d(1) * d(3) * d(8)
		     - d(2) * d(4) * d(6)); 
	}
	
	template< typename Matrix>
	__forceinline const Matrix inverse( const Matrix & d){
		const float det = 1.0/determinant3( d);
		return inverse( d, det);
	}
	
	template< typename Vector, typename Matrix>
	__forceinline Vector vector_matrix( const Vector& v, const Matrix& m ) {
	  return Vector( v[0] * m(0,0) + v[1] * m(1,0) + v[2] * m(2,0),
	                 v[0] * m(0,1) + v[1] * m(1,1) + v[2] * m(2,1),
	                 v[0] * m(0,2) + v[1] * m(1,2) + v[2] * m(2,2) );
	}
	
	template< typename Vector, typename Matrix>
	__forceinline Vector matrix_vector( const Matrix& m, const Vector& v ){
	   Vector res = v;
	   res[ 0] = v[0] * m(0,0) + v[1] * m(0,1) + v[2] * m(0,2);
	   res[ 1] = v[0] * m(1,0) + v[1] * m(1,1) + v[2] * m(1,2);
	   res[ 2] = v[0] * m(2,0) + v[1] * m(2,1) + v[2] * m(2,2);
	   return res;
	} 

	// moved from EigenDecomp.hpp

	// Jacobi iteration for the solution of eigenvectors/eigenvalues of a nxn
	// real symmetric matrix. Square nxn matrix a; size of matrix in n;
	// output eigenvalues in w; and output eigenvectors in v. Resulting
	// eigenvalues/vectors are sorted in decreasing order; eigenvectors are
	// normalized.
	// TODO: Remove this
	#define VTK_ROTATE(a,i,j,k,l) g=a[i][j];h=a[k][l];a[i][j]=g-s*(h+g*tau);\
	        a[k][l]=h+s*(g-h*tau)

	//TODO: Refactor this method into subroutines
	//use a namespace { }  with no name to
	//contain subroutines so that the compiler
	//automatically __forceinlines them.

	template< typename Matrix, typename Vector>
	void EigenDecomp( const Matrix & _a,
			  float w[3],
			  Vector v[3] ) {
	  const int MAX_ROTATIONS = 20;
	  const float one_ninth = 1./9;
	  int i, j, k, iq, ip, numPos;
	  float tresh, theta, tau, t, sm, s, h, g, c, tmp;
	  float b[3], z[3];
	  Matrix a( _a);

	  // initialize
	  for (ip=0; ip<3; ip++) {
	    for (iq=0; iq<3; iq++){
	      v[ip][iq] = 0.0;
	    }
	    v[ip][ip] = 1.0;
	  }
	  for (ip=0; ip<3; ip++) {
	    b[ip] = w[ip] = a[ip][ip];
	    z[ip] = 0.0;
	  }

	  // begin rotation sequence
	  for (i=0; i<MAX_ROTATIONS; i++){
	    sm = 0.0;
	    for (ip=0; ip<2; ip++){
	      for (iq=ip+1; iq<3; iq++){ sm += fabs(a[ip][iq]); }
	    }

	    if ( sm == 0.0 ){ break; }
	    // first 3 sweeps
	    tresh = (i < 3)? 0.2*sm*one_ninth : 0.0;
	    for (ip=0; ip<2; ip++) {
	      for (iq=ip+1; iq<3; iq++) {
	        g = 100.0*fabs(a[ip][iq]);

	        // after 4 sweeps
	        if ( i > 3 && (fabs(w[ip])+g) == fabs(w[ip])
	             && (fabs(w[iq])+g) == fabs(w[iq])) {
	          a[ip][iq] = 0.0;
	  }
	        else if ( fabs(a[ip][iq]) > tresh) {
	          h = w[iq] - w[ip];
	          if ( (fabs(h)+g) == fabs(h)){ t = (a[ip][iq]) / h; }
	          else {
	            theta = 0.5*h / (a[ip][iq]);
	            t = 1.0 / (fabs(theta)+sqrt(1.0+theta*theta));
	            if (theta < 0.0) { t = -t;}
	          }
	          c = 1.0 / sqrt(1+t*t);
	          s = t*c;
	          tau = s/(1.0+c);
	          h = t*a[ip][iq];
	          z[ip] -= h;
	          z[iq] += h;
	          w[ip] -= h;
	          w[iq] += h;
	          a[ip][iq]=0.0;
	          // ip already shifted left by 1 unit
	          for (j = 0;j <= ip-1;j++) { VTK_ROTATE(a,j,ip,j,iq); }
	          // ip and iq already shifted left by 1 unit
	          for (j = ip+1;j <= iq-1;j++) { VTK_ROTATE(a,ip,j,j,iq); }
	          // iq already shifted left by 1 unit
	          for (j=iq+1; j<3; j++) { VTK_ROTATE(a,ip,j,iq,j); }
	          for (j=0; j<3; j++) { VTK_ROTATE(v,j,ip,j,iq); }
	          }
	        }
	      }

	    for (ip=0; ip<3; ip++) {
	      b[ip] += z[ip];
	      w[ip] = b[ip];
	      z[ip] = 0.0;
	    }
	  }

	  //// this is NEVER called
	  if ( i >= MAX_ROTATIONS ) {
	      std::cerr << "Matrix3D: Error extracting eigenfunctions" << std::endl;
	  }

	  // sort eigenfunctions                 these changes do not affect accuracy
	  for (j=0; j<2; j++){                  // boundary incorrect
	    k = j;
	    tmp = w[k];
	    for (i=j+1; i<3; i++){                // boundary incorrect, shifted already
	      if (w[i] >= tmp){                  // why exchage if same?
	        k = i;
	        tmp = w[k];
	        }
	    }
	    if (k != j){
	      w[k] = w[j];
	      w[j] = tmp;
	      for (i=0; i<3; i++){
	        tmp = v[i][j];
	        v[i][j] = v[i][k];
	        v[i][k] = tmp;
	        }
	    }
	  }
	  // insure eigenvector consistency (i.e., Jacobi can compute vectors that
	  // are negative of one another (.707,.707,0) and (-.707,-.707,0). This can
	  // reek havoc in hyperstreamline/other stuff. We will select the most
	  // positive eigenvector.
	/*   int ceil_half_n = (3 >> 1) + (3 & 1); */
	/*   for (j=0; j<3; j++) { */
	/*     for (numPos=0, i=0; i<3; i++) { */
	/*       if ( v[i][j] >= 0.0 ) { numPos++; } */
	/*     } */
	/* //    if ( numPos < ceil(float(n)/float(2.0)) ) */
	/*     if ( numPos < ceil_half_n) { */
	/*       for(i=0; i<3; i++) { v[i][j] *= -1.0; } */
	/*     } */
	/*   } */
	}

	template<typename Matrix, typename Vector>
	  void EigenDecomp( const Matrix & _a,
			    float w[3],
			    Vector &v0,
			    Vector &v1,
			    Vector &v2)
	{
	  Vector temp[3];
	  EigenDecomp(_a, w, temp);
	  v0 = temp[0];
	  v1 = temp[1];
	  v2 = temp[2];
	}
			   
} //namespace Matrix

class Matrix3  {
  //TODO: std::array when we can use C++11
  float d[9];

public:
  //Default Constructor
  __forceinline Matrix3(){
	for(int i = 0; i < 9; ++i){ d[ i] = 0; }
  }
  //TODO: Deprecate this.
  //Then we can go from three Constructors to one. 
  __forceinline Matrix3( float diagonal ){ 
      d[0] = d[4] = d[8] = diagonal;
      d[1] = d[2] = d[3] = 0.0;
      d[5] = d[6] = d[7] = 0.0;
  }
  __forceinline Matrix3( const Vec3fa & diagonal ){ 
      d[0] = diagonal[0];
      d[4] = diagonal[1],
      d[8] = diagonal[2];
      d[1] = d[2] = d[3] = 0.0;
      d[5] = d[6] = d[7] = 0.0;
  }
  //TODO: not strictly correct as the Matrix3 object
  //is a float d[ 9] so the only valid model of T is
  //float, or any refinement (int, float) 
  //*but* it doesn't really matter anything else
  //will fail to compile.
  template< typename T> 
  __forceinline Matrix3( const std::vector< T> & diagonal ){ 
      d[0] = diagonal[0];
      d[4] = diagonal[1],
      d[8] = diagonal[2];
      d[1] = d[2] = d[3] = 0.0;
      d[5] = d[6] = d[7] = 0.0;
  }

__forceinline Matrix3( float v00, float v01, float v02,
                float v10, float v11, float v12,
                float v20, float v21, float v22 ){
    d[0] = v00; d[1] = v01; d[2] = v02;
    d[3] = v10; d[4] = v11; d[5] = v12;
    d[6] = v20; d[7] = v21; d[8] = v22;
}

  //Copy constructor 
  Matrix3 ( const Matrix3 & f){
	for(int i = 0; i < 9; ++i) { d[ i] = f.d[ i]; }
  }
  //Weird constructors 
  template< typename Vector> 
  __forceinline Matrix3(   const Vector & row0,
                    const Vector & row1,
                    const Vector & row2,
		    bool as_cols = false) {
    if(as_cols) {
      for(std::size_t i = 0; i < 3; ++i){
	d[ 3*i] = row0[ i];
	d[ 3*i+1]= row1[ i];
	d[ 3*i+2] = row2[ i];
      }
    }
    else {
      for(std::size_t i = 0; i < 3; ++i){
	d[ i] = row0[ i];
	d[ i+3]= row1[ i];
	d[ i+6] = row2[ i];
      }
    }
  }
  
  __forceinline Matrix3( const float* v ){ 
      d[0] = v[0]; d[1] = v[1]; d[2] = v[2];
      d[3] = v[3]; d[4] = v[4]; d[5] = v[5]; 
      d[6] = v[6]; d[7] = v[7]; d[8] = v[8];
  }
  
  __forceinline Matrix3& operator=( const Matrix3& m ){
      d[0] = m.d[0]; d[1] = m.d[1]; d[2] = m.d[2];
      d[3] = m.d[3]; d[4] = m.d[4]; d[5] = m.d[5];
      d[6] = m.d[6]; d[7] = m.d[7]; d[8] = m.d[8];
      return *this;
  }
  
  __forceinline Matrix3& operator=( const float* v ){ 
      d[0] = v[0]; d[1] = v[1]; d[2] = v[2];
      d[3] = v[3]; d[4] = v[4]; d[5] = v[5]; 
      d[6] = v[6]; d[7] = v[7]; d[8] = v[8];
      return *this;
 }

  __forceinline float* operator[]( size_t i ){ return d + 3*i; }
  __forceinline const float* operator[]( size_t i ) const{ return d + 3*i; }
  __forceinline float& operator()(size_t r, size_t c) { return d[3*r+c]; }
  __forceinline float operator()(size_t r, size_t c) const { return d[3*r+c]; }
  __forceinline float& operator()(size_t i) { return d[i]; }
  __forceinline float operator()(size_t i) const { return d[i]; }
  
    // get pointer to array of nine floats
  __forceinline float* array()
      { return d; }
  __forceinline const float* array() const
      { return d; }

  // get row from matrix
  __forceinline       Vec3fa row(const size_t& i)       { return Vec3fa(d[3*i+0],d[3*i+1],d[3*i+2]); } 
  // get row from matrix
  __forceinline const Vec3fa row(const size_t& i) const { return Vec3fa(d[3*i+0],d[3*i+1],d[3*i+2]); } 
  
  // get row from matrix
  __forceinline       Vec3fa col(const size_t& i)       { return Vec3fa(d[i],d[3+i],d[6+i]); } 

  // get row from matrix
  __forceinline const Vec3fa col(const size_t& i) const { return Vec3fa(d[i],d[3+i],d[6+i]); } 

  __forceinline Matrix3& operator+=( const Matrix3& m ){
      d[0] += m.d[0]; d[1] += m.d[1]; d[2] += m.d[2];
      d[3] += m.d[3]; d[4] += m.d[4]; d[5] += m.d[5];
      d[6] += m.d[6]; d[7] += m.d[7]; d[8] += m.d[8];
      return *this;
  }
  
  __forceinline Matrix3& operator-=( const Matrix3& m ){
      d[0] -= m.d[0]; d[1] -= m.d[1]; d[2] -= m.d[2];
      d[3] -= m.d[3]; d[4] -= m.d[4]; d[5] -= m.d[5];
      d[6] -= m.d[6]; d[7] -= m.d[7]; d[8] -= m.d[8];
      return *this;
  }
  
  __forceinline Matrix3& operator*=( float s ){
      d[0] *= s; d[1] *= s; d[2] *= s;
      d[3] *= s; d[4] *= s; d[5] *= s;
      d[6] *= s; d[7] *= s; d[8] *= s;
      return *this;
 }
  
  __forceinline Matrix3& operator/=( float s ){
      d[0] /= s; d[1] /= s; d[2] /= s;
      d[3] /= s; d[4] /= s; d[5] /= s;
      d[6] /= s; d[7] /= s; d[8] /= s;
      return *this;
  }
 
  __forceinline Matrix3& operator*=( const Matrix3& m ){
	(*this) = Matrix::mmult3((*this),m); 
	return *this;
  }
  
  __forceinline float determinant() const{
  	return Matrix::determinant3( *this);
  }
 
  __forceinline Matrix3 inverse() const { 
	const float i = 1.0/determinant();
	return Matrix::inverse( *this, i); 
  }
  __forceinline Matrix3 inverse( float i ) const {
  	return Matrix::inverse( *this, i); 
  }
  
  __forceinline bool positive_definite() const{
  	float tmp;
  	return positive_definite( tmp);
  }
  
  __forceinline bool positive_definite( float& det ) const{
	  return Matrix::positive_definite( *this, det);
  }
  
  __forceinline Matrix3 transpose() const{ return Matrix::transpose( *this); }
  
  __forceinline bool invert() {
    float i = 1.0 / determinant();
    if (!finite(i) || fabs(i) < std::numeric_limits<float>::epsilon())
      return false;
    *this = inverse( i );
    return true;
  }
    // Calculate determinant of 2x2 submatrix composed of the
    // elements not in the passed row or column.
  __forceinline float subdet( int r, int c ) const{
	const int r1 = (r+1)%3, r2 = (r+2)%3;
	const int c1 = (c+1)%3, c2 = (c+2)%3;
	assert(r >= 0 && c >= 0);
	if (r < 0 || c < 0) return DBL_MAX;
	return d[3*r1+c1]*d[3*r2+c2] - d[3*r1+c2]*d[3*r2+c1];
  }
}; //class Matrix3

__forceinline Matrix3 operator+( const Matrix3& a, const Matrix3& b ){ 
	return Matrix3(a) += b; 
}
__forceinline Matrix3 operator-( const Matrix3& a, const Matrix3& b ){ 
	return Matrix3(a) -= b; 
}

__forceinline Matrix3 operator*( const Matrix3& a, const Matrix3& b ) {
	return Matrix::mmult3( a, b);
}

template< typename Vector>
__forceinline Matrix3 outer_product( const Vector & u,
                              const Vector & v ) {
  return Matrix3( u[0] * v[0], u[0] * v[1], u[0] * v[2],
                  u[1] * v[0], u[1] * v[1], u[1] * v[2],
                  u[2] * v[0], u[2] * v[1], u[2] * v[2] );
}

template< typename T>
__forceinline std::vector< T> operator*( const Matrix3&m, const std::vector< T> & v){
		return Matrix::matrix_vector( m, v);
}

template< typename T>
__forceinline std::vector< T> operator*( const std::vector< T>& v, const Matrix3&m){
		return Matrix::vector_matrix( v, m);
}

__forceinline Vec3fa operator*( const Matrix3&m,  const Vec3fa& v){
		return Matrix::matrix_vector( m, v);
}

__forceinline Vec3fa operator*( const Vec3fa& v, const Matrix3& m){
		return Matrix::vector_matrix( v, m);
}


inline std::ostream& operator<<( std::ostream& s, const Matrix3& m ){
  return s <<  "| " << m(0,0) << " " << m(0,1) << " " << m(0,2) 
           << " | " << m(1,0) << " " << m(1,1) << " " << m(1,2) 
           << " | " << m(2,0) << " " << m(2,1) << " " << m(2,2) 
           << " |" ;
}

