#pragma once

#include "BVH.h"
#include "OBVH.h"

typedef RayT<Vec3da, double, moab::EntityHandle> MBRay;

#ifdef OBBS
typedef OBVH<Vec3da, double, moab::EntityHandle> MBVH;
#else
typedef BVH<Vec3da, double, moab::EntityHandle> MBVH;
#endif

