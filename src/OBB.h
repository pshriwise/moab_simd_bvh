
#pragma once

#include "Vec3.h"
#include "Vec3fa.h"
#include "Ray.h"
#include "mat3.h"

// Oriented Bounding Box Tree
struct OBB {

  // member variables
  Vec3fa cen;
  Vec3fa ax0, ax1, ax2;
  
  // constructors

  __forceinline OBB() : cen(zero), ax0(zero), ax1(zero), ax2(zero) { }

  __forceinline OBB(const Vec3fa cen,
		    const Vec3fa& ax0,
		    const Vec3fa& ax1,
		    const Vec3fa& ax2)
                   :cen(cen),
                    ax0(ax0),
                    ax1(ax1),
                    ax2(ax2) { }    

  // copy constructor
    __forceinline OBB ( const OBB& other ) {
      cen = other.cen;
      ax0 = other.ax0;
      ax1 = other.ax1;
      ax2 = other.ax2;
    }
  
  // assignment operator
  __forceinline OBB& operator=( const OBB& other ) {
    cen = other.cen;
    ax0 = other.ax0;
    ax1 = other.ax1;
    ax2 = other.ax2;
    return *this;
  }
    
  __forceinline bool isValid() {
    return ax0.length() + ax0.length() + ax2.length() > 0;
  }

  // property methods
  __forceinline Vec3fa center() { return cen; }

  __forceinline Vec3fa center2() { return 2.0f * cen; }
  
  __forceinline Vec3fa size() const { return 2.0f * Vec3fa(ax0.length(), ax1.length(), ax2.length()); }

  __forceinline float inner_radius() const { return std::min(ax0.length(), std::min(ax1.length(), ax2.length())); }

  __forceinline float outer_radius() const { return std::max(ax0.length(), std::max(ax1.length(), ax2.length())); }

  __forceinline bool point_in_box( const Vec3fa& point ) const {
    Vec3fa from_center = point - cen;

  
    float lensq;

    lensq = ax0.length_sqr();
    if(fabs(dot(from_center, ax0)) > lensq) return false;
    lensq = ax1.length_sqr();
    if(fabs(dot(from_center, ax1)) > lensq) return false;
    lensq = ax2.length_sqr();
    if(fabs(dot(from_center, ax2)) > lensq) return false;
    
    return true;
  }

  
};

__forceinline bool inside(const OBB& box, const Vec3fa& pnt) {
    return box.point_in_box(pnt);
  }

// property functions
__forceinline float volume( const OBB& box) { return reduce_mul(box.size()); }

__forceinline float halfArea( const OBB& box ) { return halfArea(box.size()); }

__forceinline float area(const OBB& box) { return 2.0f * halfArea(box); }


inline bool check_ray_limits(const float  normal_par_pos,
                             const float  normal_par_dir,
                             const float  half_extent,
                             const float* nonneg_ray_len,
                             const float* neg_ray_len ) {

  const float extent_pos_diff = half_extent - normal_par_pos;

  // limit in positive direction
  if(nonneg_ray_len) { // should be 0 <= t <= nonneg_ray_len
    assert(0 <= *nonneg_ray_len);
    if       (normal_par_dir>0) { // if/else if needed for pos/neg divisor
      if(*nonneg_ray_len*normal_par_dir>=extent_pos_diff && extent_pos_diff>=0) return true;
    } else if(normal_par_dir<0) {
      if(*nonneg_ray_len*normal_par_dir<=extent_pos_diff && extent_pos_diff<=0) return true;
    }
  } else {            // should be 0 <= t
    if       (normal_par_dir>0) { // if/else if needed for pos/neg divisor
      if(extent_pos_diff>=0) return true;
    } else if(normal_par_dir<0) {
      if(extent_pos_diff<=0) return true;
    }
  }

  // limit in negative direction
  if(neg_ray_len) {   // should be neg_ray_len <= t < 0
    assert(0 >= *neg_ray_len);
    if       (normal_par_dir>0) { // if/else if needed for pos/neg divisor
      if(*neg_ray_len*normal_par_dir<=extent_pos_diff && extent_pos_diff<0) return true;
    } else if(normal_par_dir<0) {
      if(*neg_ray_len*normal_par_dir>=extent_pos_diff && extent_pos_diff>0) return true;
    }
  }

  return false;
}

template<typename V, typename P, typename I>
  bool ray_intersection( const OBB& box, const RayT<V,P,I> &ray ) {

  const Vec3fa cx = box.cen - ray.org;
  const float  dist_s = dot(cx,ray.dir);
  const float dist_sq = dot(cx,cx) - (dist_s*dist_s);
  const float max_diagsq = box.outer_radius()*box.outer_radius();

  // For the largest sphere, no intersections exist if discriminant is negative.
  // Geometrically, if distance from box center to line is greater than the 
  // longest diagonal, there is no intersection.
  // manipulate the discriminant: 0 > dist_s*dist_s - cx%cx + max_diagsq
  if(dist_sq > max_diagsq) return false;

  //get transpose of the axes
  Matrix3 B = Matrix3(box.ax0, box.ax1, box.ax2, true).transpose();

  // transform ray to box coordinate system
  Vec3fa par_pos = B * -cx;
  Vec3fa par_dir = B * ray.dir;

  // (ax0.length() is half of box width along axis 0)
  const float half_x = box.ax0.length();
  const float half_y = box.ax1.length();
  const float half_z = box.ax2.length();

  // test if ray_origin is inside box
  if (par_pos[0] <= half_x && par_pos[0] >= -half_x &&
      par_pos[1] <= half_y && par_pos[1] >= -half_y &&
      par_pos[2] <= half_z && par_pos[2] >= -half_z)
    return true;
  
  // Fast Rejection Test: Ray will not intersect if it is going away from the box.
  // This will not work for rays with neg_ray_len.
  if ((par_pos[0] >  half_x && par_dir[0] >= 0) ||
      (par_pos[0] < -half_x && par_dir[0] <= 0))
    return false;
  
  if ((par_pos[1] >  half_y && par_dir[1] >= 0) ||
      (par_pos[1] < -half_y && par_dir[1] <= 0))
    return false;
  
  if ((par_pos[2] >  half_z && par_dir[2] >= 0) ||
      (par_pos[2] < -half_z && par_dir[2] <= 0))
    return false;

  // the slow part, planar checks
  
  //test two xy plane
  if (fabs(par_dir[0] * (half_z - par_pos[2]) + par_dir[2] * par_pos[0]) 
        <= fabs(par_dir[2] * half_x) &&  // test against x extents using z
      fabs(par_dir[1] * (half_z - par_pos[2]) + par_dir[2] * par_pos[1]) 
        <= fabs(par_dir[2] * half_y) &&  // test against y extents using z
      check_ray_limits( par_pos[2], par_dir[2], half_z, &ray.tfar, &ray.tnear ) )
    return true;
  if (fabs(par_dir[0] * (-half_z - par_pos[2]) + par_dir[2] * par_pos[0]) 
        <= fabs(par_dir[2] * half_x) && 
      fabs(par_dir[1] * (-half_z - par_pos[2]) + par_dir[2] * par_pos[1]) 
        <= fabs(par_dir[2] * half_y) &&
      check_ray_limits( par_pos[2], par_dir[2], -half_z, &ray.tfar, &ray.tnear ) )
    return true;

    //test two xz plane
  if (fabs(par_dir[0] * (half_y - par_pos[1]) + par_dir[1] * par_pos[0]) 
        <= fabs(par_dir[1] * half_x) && 
      fabs(par_dir[2] * (half_y - par_pos[1]) + par_dir[1] * par_pos[2]) 
        <= fabs(par_dir[1] * half_z) &&
      check_ray_limits( par_pos[1], par_dir[1], half_y, &ray.tfar, &ray.tnear ) )
    return true;
  if (fabs(par_dir[0] * (-half_y - par_pos[1]) + par_dir[1] * par_pos[0]) 
        <= fabs(par_dir[1] * half_x) && 
      fabs(par_dir[2] * (-half_y - par_pos[1]) + par_dir[1] * par_pos[2])
        <= fabs(par_dir[1] * half_z) &&
      check_ray_limits( par_pos[1], par_dir[1], -half_y, &ray.tfar, &ray.tnear ) )
    return true;

    //test two yz plane
  if (fabs(par_dir[1] * (half_x - par_pos[0]) + par_dir[0] * par_pos[1]) 
        <= fabs(par_dir[0] * half_y) &&
      fabs(par_dir[2] * (half_x - par_pos[0]) + par_dir[0] * par_pos[2]) 
        <= fabs(par_dir[0] * half_z) &&
      check_ray_limits( par_pos[0], par_dir[0], half_x, &ray.tfar, &ray.tnear ) )
    return true;
  if (fabs(par_dir[1] * (-half_x - par_pos[0]) + par_dir[0] * par_pos[1])
        <= fabs(par_dir[0] * half_y) &&
      fabs(par_dir[2] * (-half_x - par_pos[0]) + par_dir[0] * par_pos[2]) 
        <= fabs(par_dir[0] * half_z) &&
      check_ray_limits( par_pos[0], par_dir[0], -half_x, &ray.tfar, &ray.tnear ) )
    return true;

  return false;
}

