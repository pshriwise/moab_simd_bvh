#pragma once


#include "Vec3.h"
#include "constants.h"

struct Ray {

  /* Empty Constructor */
  inline Ray() {}
  
  /* Ray Constructor */
  inline Ray(const Vec3f& org, const Vec3f &dir,
	     const float& tnear = zero, const float& tfar = inf,
	     const int mask = -1)
    : org(org), dir(dir), tnear(tnear), tfar(tfar), mask(mask), geomID(-1), primID(-1), instID(-1) {}

  /* Ray data */
  Vec3f org;
  Vec3f dir;
  float tnear;
  float tfar;
  int mask;

  /* Hit data */
  Vec3f Ng; // tri normal
  float u; // barycentric coordinate of hit
  float v; // barycentric coordinate of hit
  int geomID; // geometry ID (equivalent to surface ID for us)
  int primID; // triangle ID (equivalent to triangle EntityHandle)
  int instID; // kernel instance ID (might be able to replace with volume EntityHandle

};

  struct TravRay {

    /* Empty constructor */
    inline TravRay() {}

    inline TravRay(const Vec3f &ray_org, const Vec3f& ray_dir)
      : org_xyz(ray_org), dir_xyz(ray_dir) {
      rdir = rcp(dir_xyz);
      org = ray_org;
      dir = ray_dir;
      nearX = ray_dir.x >= 0.0f ? 0*sizeof(float) : 1*sizeof(float);
      nearY = ray_dir.y >= 0.0f ? 2*sizeof(float) : 3*sizeof(float);
      nearZ = ray_dir.z >= 0.0f ? 4*sizeof(float) : 5*sizeof(float);
      farX  = nearX ^ sizeof(float);
      farY  = nearY ^ sizeof(float);
      farZ  = nearZ ^ sizeof(float);
    }

    
    Vec3f org_xyz, dir_xyz;
    Vec3f org, dir, rdir;
    size_t nearX, nearY, nearZ;
    size_t farX, farY, farZ;
  };
