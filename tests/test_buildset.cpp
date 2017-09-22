


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

  BuildPrimitive p = BuildPrimitive();

  std::vector<BuildPrimitive> v;
  v.push_back(p);

  s = Set(v);

  CHECK_EQUAL( 1, s.prims.size() );
  CHECK_EQUAL( 1, (int)s.size() );
  
  
  return;
}
