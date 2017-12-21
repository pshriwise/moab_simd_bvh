#pragma once

#include "Builder.h"
#include "Intersector.h"
#include "TriangleRef.h"

typedef BVHIntersectorT<TriangleRef, Vec3fa, float, int> TriIntersector;
typedef BVHIntersectorT<TriangleRef, Vec3da, double, int> DblTriIntersector;
typedef BuildStateT<TriangleRef> BuildStateTri;
typedef BVHBuilder<TriangleRef> TriangleBVH;
