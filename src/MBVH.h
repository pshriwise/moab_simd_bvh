#pragma once

#include "BVH.h"
#include "OBVH.h"
#include "MixedBVH.h"

typedef RayT<Vec3da, double, moab::EntityHandle> MBRay;

#ifdef MIXED
typedef MixedBVH<Vec3da, double, moab::EntityHandle> MBVH;
#endif

#ifdef OBBS
typedef OBVH<Vec3da, double, moab::EntityHandle> MBVH;
#endif

#ifdef AABBS
typedef BVH<Vec3da, double, moab::EntityHandle> MBVH;
#endif

