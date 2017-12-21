#pragma once

#include "BVH.h"

/* typedef BVHBuilder<TriangleRef> TriangleBVH; */

/* typedef BuildStateT<TriangleRef> BuildStateTri; */

/* typedef BVHIntersectorT<TriangleRef, Vec3fa, float, int> TriIntersector; */

/* typedef BVHIntersectorT<TriangleRef, Vec3da, double, int> DblTriIntersector; */

typedef RayT<Vec3da, double, moab::EntityHandle> MBRay;
typedef BVH<Vec3da, double, moab::EntityHandle, MBTriangleRef> MBVH;
