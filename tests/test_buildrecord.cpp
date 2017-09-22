
#include <stdio.h>

#include "testutil.hpp"
#include "BuildState.h"

void test_buildrecord();

int main(int argc, char** argv) {

  test_buildrecord();

  return 0;
}

void test_buildrecord() {

  BuildState b = BuildState();

  BuildState* ptr = new BuildState();

  BuildPrimitive p = BuildPrimitive();

  std::vector<BuildPrimitive> v;
  v.push_back(p);

  size_t test_depth = 20, test_size = 1;
  b = BuildState(test_depth,v);

  CHECK_EQUAL(test_depth, b.depth);

  CHECK_EQUAL(test_size, b.size());

  CHECK( b.ptr()[0] == b.prims[0] );
  
  float corners[6] = {-1.0, -1.0, -1.0, 1.0, 1.0, 1.0 };

  BuildPrimitive p1 = BuildPrimitive(corners[0], corners[1], corners[2], 1,
				     corners[3], corners[4], corners[5], 1);

  v.push_back(p1);
  
  BuildState c = BuildState(v);

  CHECK( c > b );
  CHECK( b < c );

  return;
}
