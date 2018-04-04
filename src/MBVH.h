#pragma once

#include "BVH.h"
#include "OBVH.h"

typedef RayT<Vec3da, double, moab::EntityHandle> MBRay;
typedef BVH<Vec3da, double, moab::EntityHandle> MBVH;

typedef OBVH<Vec3da, double, moab::EntityHandle> OMBVH;
