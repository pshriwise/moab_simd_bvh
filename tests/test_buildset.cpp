


#include <stdio.h>

#include "testutil.hpp"
#include "BuildState.h"

void test_buildset();

int main(int argc, char** argv) {

  test_buildset();

  return 0;
}


void test_buildset() {

    // test data
  float corners[6] = {-1.0, -1.0, -1.0, 1.0, 1.0, 1.0 };
  
  BuildPrimitive p = BuildPrimitive(corners[0], corners[1], corners[2], 0,
				    corners[3], corners[4], corners[5], 0);

  AABB test_box = AABB(corners);

  // setup vector
  std::vector<BuildPrimitive> v;
  v.push_back(p);

  // empty set object
  Set s = Set();
  CHECK_EQUAL(0, (int)s.size());

  // testing push back
  s.push_back(p);
  CHECK_EQUAL( 1, (int)s.prims.size() );
  CHECK_EQUAL( 1, (int)s.size() );
  CHECK(s.prims[0] == s.ptr()[0]);
  CHECK(s.bounds() == test_box);

  // clear out the set
  s.clear();
  CHECK_EQUAL(0, (int)s.size());
  CHECK(s.prims[0] == s.ptr()[0]);

  // create using vector constructor
  s = Set(v);
  CHECK_EQUAL( 1, (int)s.prims.size() );
  CHECK_EQUAL( 1, (int)s.size() );
  CHECK(s.prims[0] == s.ptr()[0]);
  CHECK(s.bounds() == test_box);

  // clear out the set again
  s.clear();
  CHECK_EQUAL(0, (int)s.size());
  CHECK(s.prims[0] == s.ptr()[0]);

  return;
}
