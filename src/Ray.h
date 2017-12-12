#pragma once


#include "Vec3fa.h"
#include "Vec3da.h"

#include "constants.h"
#include "vfloat.h"

template<typename V, typename P, typename I>
struct RayT {

  /* Empty Constructor */
  inline RayT() {}
  
  /* RayT Constructor */
  inline RayT(const V& org, const V &dir,
	     const P& tnear = zero, const P& tfar = inf,
	     const int mask = -1)
    : org(org), dir(dir), tnear(tnear), tfar(tfar), mask(mask), geomID(-1), primID(-1), instID(-1), u(0.0f), v(0.0f) { Ng = V(); }


  
  inline bool valid() const {
    bool vx = (fabs(org.x) <= FLT_LARGE) & (fabs(dir.x) <= FLT_LARGE);
    bool vy = (fabs(org.y) <= FLT_LARGE) & (fabs(dir.y) <= FLT_LARGE);
    bool vz = (fabs(org.z) <= FLT_LARGE) & (fabs(dir.z) <= FLT_LARGE);
    bool vn = fabs(tnear) <= (P)inf;
    bool vf = fabs(tnear) <= (P)inf;
    return vx & vy & vz & vn & vf;
  }

  /* RayT data */
  V org;
  V dir;
  P tnear;
  P tfar;
  int mask;

  /* Hit data */
  V Ng; // tri normal
  P u; // barycentric coordinate of hit
  P v; // barycentric coordinate of hit
  I geomID; // geometry ID (equivalent to surface ID for us)
  I primID; // triangle ID (equivalent to triangle EntityHandle)
  I instID; // kernel instance ID (might be able to replace with volume EntityHandle

};

template<typename v, typename p, typename i>
  std::ostream& operator <<(std::ostream &os, RayT<v,p,i> const &r) {
  return os << "Origin: " << r.org << std::endl
            << "Direction: " << r.dir << std::endl
            << "tNear: " << r.tnear << std::endl
            << "tFar: " << r.tfar << std::endl
 	    << "--------" << std::endl
            << "Hit Info" << std::endl
            << "--------" << std::endl
	    << "Triangle Normal: " << r.Ng << std::endl
            << "Barycentric Coords: " << "[" << r.u << " " << r.v << " " << 1-r.u-r.v << "]" << std::endl
    	    << "Instance ID: " << r.instID << std::endl
	    << "Primitive ID: " << r.primID << std::endl
            << "Geometry ID: " << r.geomID << std::endl;
}

typedef RayT<Vec3fa, float, int> Ray;
typedef RayT<Vec3da, double, int> dRay;

struct TravRay {

    /* Empty constructor */
    inline TravRay() {}

    inline TravRay(const Vec3fa &ray_org, const Vec3fa &ray_dir)
      : org_xyz(ray_org), dir_xyz(ray_dir) {
      rdir = rcp_safe(dir_xyz);
      org = ray_org;
      dir = ray_dir;
      nearX = ray_dir.x >= 0.0f ? 0*sizeof(vfloat4) : 1*sizeof(vfloat4);
      nearY = ray_dir.y >= 0.0f ? 2*sizeof(vfloat4) : 3*sizeof(vfloat4);
      nearZ = ray_dir.z >= 0.0f ? 4*sizeof(vfloat4) : 5*sizeof(vfloat4);
      farX  = nearX ^ sizeof(vfloat4);
      farY  = nearY ^ sizeof(vfloat4);
      farZ  = nearZ ^ sizeof(vfloat4);
    }


      inline TravRay(const Vec3da &ray_org, const Vec3da &ray_dir)
      : org_xyz(ray_org), dir_xyz(ray_dir) {
      rdir = rcp_safe(dir_xyz);
      org = ray_org;
      dir = ray_dir;
      nearX = ray_dir.x >= 0.0f ? 0*sizeof(vfloat4) : 1*sizeof(vfloat4);
      nearY = ray_dir.y >= 0.0f ? 2*sizeof(vfloat4) : 3*sizeof(vfloat4);
      nearZ = ray_dir.z >= 0.0f ? 4*sizeof(vfloat4) : 5*sizeof(vfloat4);
      farX  = nearX ^ sizeof(vfloat4);
      farY  = nearY ^ sizeof(vfloat4);
      farZ  = nearZ ^ sizeof(vfloat4);
    }
  
    Vec3fa org_xyz, dir_xyz;
    Vec3fa org, dir, rdir;
    size_t nearX, nearY, nearZ;
    size_t farX, farY, farZ;
    int sense;
    unsigned setID;
  };
