#pragma once


#include "Vec3fa.h"
#include "constants.h"
#include "vfloat.h"

struct Ray {

  /* Empty Constructor */
  inline Ray() {}
  
  /* Ray Constructor */
  inline Ray(const Vec3fa& org, const Vec3fa &dir,
	     const float& tnear = zero, const float& tfar = inf,
	     const int mask = -1)
    : org(org), dir(dir), tnear(tnear), tfar(tfar), mask(mask), geomID(-1), primID(-1), instID(-1) {}

  
  inline bool valid() const {
    bool vx = (fabs(org.x) <= FLT_LARGE) & (fabs(dir.x) <= FLT_LARGE);
    bool vy = (fabs(org.y) <= FLT_LARGE) & (fabs(dir.y) <= FLT_LARGE);
    bool vz = (fabs(org.z) <= FLT_LARGE) & (fabs(dir.z) <= FLT_LARGE);
    bool vn = fabs(tnear) <= (float)inf;
    bool vf = fabs(tnear) <= (float)inf;
    return vx & vy & vz & vn & vf;
  }

  /* Ray data */
  Vec3fa org;
  Vec3fa dir;
  float tnear;
  float tfar;
  int mask;

  /* Hit data */
  Vec3fa Ng; // tri normal
  float u; // barycentric coordinate of hit
  float v; // barycentric coordinate of hit
  int geomID; // geometry ID (equivalent to surface ID for us)
  int primID; // triangle ID (equivalent to triangle EntityHandle)
  int instID; // kernel instance ID (might be able to replace with volume EntityHandle

};

inline std::ostream& operator <<(std::ostream &os, Ray const &r) {
  return os << "Origin: " << r.org << std::endl
            << "Direction: " << r.dir << std::endl
            << "tNear: " << r.tnear << std::endl
            << "tFar: " << r.tfar << std::endl
	    << "--------" << std::endl
            << "Hit Info" << std::endl
            << "--------" << std::endl
	    << "Triangle Normal: " << r.Ng << std::endl
            << "Barycentric Coords: " << Vec3fa(r.u,r.v,1-r.u-r.v) << std::endl
    	    << "Instance ID: " << r.instID << std::endl
	    << "Primitive ID: " << r.primID << std::endl
            << "Geometry ID: " << r.geomID << std::endl;
}

  struct TravRay {

    /* Empty constructor */
    inline TravRay() {}

    inline TravRay(const Vec3fa &ray_org, const Vec3fa& ray_dir)
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
  };