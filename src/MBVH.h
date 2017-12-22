#pragma once

#include "BVH.h"

typedef RayT<Vec3da, double, moab::EntityHandle> MBRay;
typedef BVH<Vec3da, double, moab::EntityHandle> MBVH;
