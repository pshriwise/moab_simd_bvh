
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
  Vec3fa box_center;
  float area;
  Matrix3 covariance;
  LinSpace transform;
    
  // constructors
  __forceinline OBB() : bbox(AABB()), covariance(0.0), transform(zero) { }

  __forceinline OBB( const Vec3fa& center,
		     const Vec3fa& size,
		     const Vec3fa& axis0,
		     const Vec3fa& axis1,
		     const Vec3fa& axis2) {
    bbox = AABB(center-(0.5*size),center+(0.5*size));
    transform = LinSpace(axis0.normalized(), axis1.normalized(), axis2.normalized()).transpose();
  }

  __forceinline OBB( float *x, float *y, float *z, size_t num_pnts) : bbox(AABB()), covariance(0.0f), transform(zero), box_center(zero) {
    // set the covariance matrix for all points
    for(size_t i = 0; i < num_pnts; i++) {
      box_center += Vec3fa(x[i], y[i], z[i]);
    }
    box_center /= num_pnts;

    for(size_t i = 0; i < num_pnts; i++) {
      Vec3fa p = Vec3fa(x[i], y[i], z[i]);
      p -= box_center;
      covariance += outer_product(p, p);
    }

    float lambda[3];
    Vec3fa axes[3];
    Matrix::EigenDecomp(covariance, lambda, axes);

    axes[0].normalize(); axes[1].normalize(); axes[2].normalize();
    
    Vec3fa min(inf), max(neg_inf);
    for(size_t i = 0; i < num_pnts; i++) {
      Vec3fa p = Vec3fa(x[i],y[i],z[i]);
      
      for(size_t j = 0; j < 3; j++) {
	float t = dot(axes[j], p);
	if ( t < min[j] ) min[j] = t;
	if ( t > max[j] ) max[j] = t;
      }
    }

    Vec3fa mid = 0.5 * (max + min);
    box_center += mid[0] * axes[0] + mid[1] * axes[1] + mid[2] * axes[2];

    bbox = AABB(min,max);
    
    transform = LinSpace(axes[0], axes[1], axes[2]);
    transform = transform.transpose();
    
    return;
  }

  __forceinline void bump(const float& bump_val) {
    bbox.bump(bump_val);
  }
  
  __forceinline void clear() {
    box_center = 0.0;
    covariance = Matrix3(0.0);
    transform = LinSpace(zero);
    bbox.clear();
  }
  
  /* __forceinline void update(const float& x, const float& y, const float& z) { */
  /*   Vec3fa point(z,y,z); */
  /*   update(point); */
  /*   return; */
  /* } */

  /* __forceinline void update(const Vec3fa& pnt) { */
  /*   update_covariance(pnt); */
  /*   update_axes(); */
  /*   update_extents(pnt); */
  /*   return; */
  /* } */

  /* __forceinline void update_covariance(const float& x, const float& y, const float& z) { */
  /*   Vec3fa point(x,y,z); */
  /*   update_covariance(point); */
  /*   return; */
  /* } */

  /* __forceinline void update_covariance(const Vec3fa& pnt) { */
    
  /*   // update axis-aligned box_center */
  /*   box_center = ((float)num_points * box_center); */
  /*   box_center += pnt; */
  /*   num_points++; */
  /*   box_center = box_center / (float)num_points; */
    
  /*   // update the covariance matrix */
  /*   Vec3fa v = pnt - box_center; */
  /*   covariance += outer_product(v,v); */
    
  /*   return; */
  /* } */

  /* __forceinline void update_axes() { */
    
  /*   // perform the Eigenvalue decomposition to determine the oriented axes */
  /*   float l[3]; */
  /*   Vec3fa ax0, ax1, ax2; */
  /*   Matrix::EigenDecomp(covariance, l, ax0, ax1, ax2); */

  /*   // normalize axes, extents are stored separately */
  /*   ax0.normalize(); ax1.normalize(); ax2.normalize(); */
    
  /*   transform = LinSpace(ax0, ax1, ax2); */
  /*   transform = transform.transpose(); */
    
  /*   return; */
  /* } */

  /* __forceinline void update_extents(const float& x, const float& y, const float& z) { */
  /*   Vec3fa point(x,y,z); */
  /*   update_extents(point); */
  /*   return; */
  /* } */
  
  /* __forceinline void update_extents(const Vec3fa& pnt) { */
  /*   Vec3fa p = xfmPnt(transform, pnt); */
  /*   bbox.update(p.x, p.y, p.z);     */
  /* } */
  
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
    Vec3fa xpnt = xfmPnt(transform, point);
    return inside(bbox, xpnt);
  }

  __forceinline OBB splitBox(const size_t& axis, const float& t_start, const float& t_end ) {
    OBB ret_box;

    ret_box.transform = this->transform;
    ret_box.bbox = this->bbox;

    ret_box.covariance = Matrix3(0.0);
    ret_box.box_center = 0.0;
    
    Vec3fa box_size;

    box_size = bbox.size();
    ret_box.bbox.lower[axis] = bbox.lower[axis] + (box_size[axis] * t_start);

    box_size = bbox.size();
    ret_box.bbox.upper[axis] = bbox.lower[axis] + (box_size[axis] * t_end);
    
    return ret_box;
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
