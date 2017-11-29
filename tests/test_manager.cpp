

#include "test_files.h"

#include "MBVHManager.h"
#include "moab/Core.hpp"


int main(int argc, char** argv) {

  moab::Interface* mbi = new moab::Core();

  moab::ErrorCode rval = mbi->load_file(TEST_CUBE);
  MB_CHK_SET_ERR(rval, "Failed to load the test file: " << TEST_CUBE);
  
  MBVHManager MBVHM(mbi);

  assert(MBVHM.MDAM);

  assert(MBVHM.BVHRoots.size() > 0);

  return 0;

}
