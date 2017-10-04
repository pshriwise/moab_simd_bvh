

#include <set>
#include <vector>

#include "Builder.h"
#include "BuildState.h"
#include "TriangleRef.h"

#include "moab/Core.hpp"

typedef BVHBuilder<TriangleRef> TriangleBVH;

typedef BuildStateT<TriangleRef> BuildStateTri;
