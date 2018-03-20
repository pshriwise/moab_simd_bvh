
#pragma once

#include "Vec3.h"
#include "Vec3fa.h"
#include "Ray.h"

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

  __forceinline Vec3fa center() { return cen; }

  __forceinline Vec3fa center2() { return 2.0f * cen; }

  __forceinline Vec3fa size() const { return Vec3fa(ax0.length(), ax1.length(), ax2.length()); }
  
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
