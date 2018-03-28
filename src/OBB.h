
#pragma once


#include "space.h"

#include "Vec3.h"
#include "Vec3fa.h"
#include "Ray.h"
#include "AABB.h"
#include "mat3.h"             

// Oriented Bounding Box Tree
struct OBB {

  // member variables
  AABB bbox;
  LinSpace transform;
    
  // constructors

  __forceinline OBB() : bbox(AABB()) { }

  __forceinline OBB( const Vec3fa& center,
		     const Vec3fa& size,
		     const Vec3fa& axis0,
		     const Vec3fa& axis1,
		     const Vec3fa& axis2) {

    bbox = AABB(center-(0.5*size),center+(0.5*size));
    transform = LinSpace(axis0.normalized(), axis1.normalized(), axis2.normalized()).transpose();
  }
		    
  __forceinline OBB( float *x, float *y, float *z, size_t num_pnts) : bbox(AABB()) {

    // calculate the center of the box
    for( size_t i = 0; i < num_pnts; i++) {
      bbox.update(x[i], y[i], z[i]);
    }
    Vec3fa cen = bbox.center();

    Matrix3 m(0.0);
    Vec3fa v;
    // compute the covariance matrix
    for( size_t i = 0; i < num_pnts; i++) {
      v.x = x[i]; v.y = y[i]; v.z = z[i];
      v -= cen;
      m += outer_product(v,v);
    }

    float l[3];
    Vec3fa ax0, ax1, ax2;
    Matrix::EigenDecomp(m, l, ax0, ax1, ax2);

    // ensure axes are the correct length
    ax0.normalize(); 
    ax1.normalize();
    ax2.normalize();

    // set point/ray transform matrix
    Matrix3 mat(ax0, ax1, ax2);
    

    // create the true box from the axes
    Vec3fa min(inf), max(neg_inf);
    for(size_t i = 0; i < num_pnts; i++) {
      // construct vec
      Vec3fa pnt(x[i], y[i], z[i]);

      // find perpindicular point on each oriented axes
      float t;
      t = dot(ax0, (pnt - cen)) / dot(ax0,ax0);
      if ( t < min[0] ) min[0] = t;
      if ( t > max[0] ) max[0] = t;
      t = dot(ax1, (pnt - cen)) / dot(ax1,ax1);
      if ( t < min[1] ) min[1] = t;
      if ( t > max[1] ) max[1] = t;
      t = dot(ax2, (pnt - cen)) / dot(ax2,ax2);
      if ( t < min[2] ) min[2] = t;
      if ( t > max[2] ) max[2] = t;
    }

    // upate the bounding box min/max
    const float bump = 5e-03;
    bbox = AABB(min-bump, max+bump);
    // set the box transform
    transform = LinSpace(ax0, ax1, ax2).transpose();
    
    return;
  }
  
  // copy constructor
  __forceinline OBB ( const OBB& other ) {
    bbox = other.bbox;
    transform = other.transform;
  }
  
  // assignment operator
  __forceinline OBB& operator=( const OBB& other ) {
    bbox = other.bbox;
    transform = other.transform;
    return *this;
  }
    
  __forceinline bool isValid() {
    return bbox.isValid();
  }

  // property methods
  __forceinline       Vec3fa center()       { return bbox.center(); }
  __forceinline const Vec3fa center() const { return bbox.center(); }

  __forceinline Vec3fa center2() { return bbox.center2(); }
  
  __forceinline Vec3fa size() const { return bbox.size(); }
  __forceinline Vec3fa halfSize() const { return 0.5f * bbox.size(); }

  __forceinline float inner_radius() const { return reduce_min(size()); }

  __forceinline float outer_radius() const { return reduce_max(size()); }

  __forceinline bool point_in_box( const Vec3fa& point ) const {
    Vec3fa from_center = point - bbox.center();
 
    Vec3fa len = halfSize();
    if(fabs(dot(from_center, transform.row0())) > len[0]) return false;
    if(fabs(dot(from_center, transform.row1())) > len[1]) return false;
    if(fabs(dot(from_center, transform.row2())) > len[2]) return false;
    
    return true;
  }

  
};

__forceinline bool inside(const OBB& box, const Vec3fa& pnt) {
    return box.point_in_box(pnt);
  }

// property functions
__forceinline float volume( const OBB& box) { return reduce_mul(box.size()); }

__forceinline float halfArea( const OBB& box ) { return halfArea(box.size()); }

__forceinline float area(const OBB& box) { return 2.0f * halfArea(box.size()); }


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

  const Vec3fa cx = box.center() - ray.org;
  const float  dist_s = dot(cx,ray.dir);
  const float dist_sq = dot(cx,cx) - (dist_s*dist_s);
  const float max_diagsq = box.outer_radius()*box.outer_radius();

  // For the largest sphere, no intersections exist if discriminant is negative.
  // Geometrically, if distance from box center to line is greater than the 
  // longest diagonal, there is no intersection.
  // manipulate the discriminant: 0 > dist_s*dist_s - cx%cx + max_diagsq
  if(dist_sq > max_diagsq) return false;

  //get transpose of the axes

  // transform ray to box coordinate system
  Vec3fa par_pos = box.transform * (ray.org - box.center());
  Vec3fa par_dir = box.transform * ray.dir;

  // (ax0.length() is half of box width along axis 0)
  const Vec3fa half_size = box.halfSize();
  const float half_x = half_size[0];
  const float half_y = half_size[1];
  const float half_z = half_size[2];

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
