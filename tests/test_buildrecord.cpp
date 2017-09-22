
#include <stdio.h>

#include "testutil.hpp"
#include "BuildState.h"

void test_buildrecord();

int main(int argc, char** argv) {

  test_buildrecord();

  return 0;
}

void test_buildrecord() {

  BuildRecord b = BuildRecord();

  BuildRecord* ptr = new BuildRecord();

  BuildPrimitive p = BuildPrimitive();

  std::vector<BuildPrimitive> v;
  v.push_back(p);

  size_t test_depth = 20, test_size = 1;
  b = BuildRecord(test_depth,v);

  CHECK_EQUAL(test_depth, b.depth);

  CHECK_EQUAL(test_size, b.size());

  return;
}
