

#include <set>
#include <vector>

#include "Builder.h"
#include "BuildState.h"
#include "Intersector.h"
#include "TriangleRef.h"

typedef BVHBuilder<TriangleRef> TriangleBVH;

typedef BuildStateT<TriangleRef> BuildStateTri;

typedef BVHIntersectorT<TriangleRef> TriIntersector;
