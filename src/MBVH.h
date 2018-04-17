#pragma once

#include "BVH.h"
#include "OBVH.h"
#include "MixedBVH.h"

typedef RayT<Vec3da, double, moab::EntityHandle> MBRay;

typedef MixedBVH<Vec3da, double, moab::EntityHandle> MBVH;

/* #ifdef OBBS */
/* typedef OBVH<Vec3da, double, moab::EntityHandle> MBVH; */
/* #else */
/* typedef BVH<Vec3da, double, moab::EntityHandle> MBVH; */
/* #endif */

