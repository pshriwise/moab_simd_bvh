


#include <stdio.h>

#include "testutil.hpp"
#include "BuildState.h"

void test_buildset();

int main(int argc, char** argv) {


  test_buildset();

  return 0;
}


void test_buildset() {

  Set s = Set();

  CHECK_EQUAL( 0, (int)s.size() );


  float corners[6] = {-1.0, -1.0, -1.0, 1.0, 1.0, 1.0 };
  
  BuildPrimitive p = BuildPrimitive(corners[0], corners[1], corners[2], 0,
				    corners[3], corners[4], corners[5], 0);

  std::vector<BuildPrimitive> v;
  v.push_back(p);

  s.push_back(p);

  CHECK_EQUAL( 1, (int)s.prims.size() );
  CHECK_EQUAL( 1, (int)s.size() );

  AABB test_box = AABB(corners);
  AABB box = s.bounds();

  std::cout << box << std::endl;
  std::cout << test_box << std::endl;
  CHECK(box == test_box);

  
  return;
}
