


#include "Vec3.h"


struct Ray {

  /* Empty Constructor */
  inline Ray() {}
  
  /* Ray Constructor */
  inline Ray(const Vec3f& org, const Vec3f &dir,
	     const float& tnear = zero, const float& tfar = inf,
	     const float& time = zero, const int mask = -1)
    : org(org), dir(dir), tnear(tnear), tfar(tfar), time(time), mask(mask), geomID(-1), primID(-1), instID(-1) {}

  /* Ray data */
  Vec3f org;
  Vec3f dir;
  float tnear;
  float tfar;
  int mask;

  /* Hit data */
  Vec3 Ng; // tri normal
  float u; // barycentric coordinate of hit
  float v; // barycentric coordinate of hit
  int geomID; // geometry ID (equivalent to surface ID for us)
  int primID; // triangle ID (equivalent to triangle EntityHandle)
  int instID; // kernel instance ID (might be able to replace with volume EntityHandle

}
