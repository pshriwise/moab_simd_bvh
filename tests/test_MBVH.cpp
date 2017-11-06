#include "test_files.h"

#include "assert.h"

#include "moab/Core.hpp"
#include "moab/CartVect.hpp"

#include "MBVH.h"

#define EPS 1e-06f

#include "testutil.hpp"

int main( int argc, char** argv ) {

  moab::Interface* mbi = new moab::Core();
  
  moab::ErrorCode rval;
  
  rval = mbi->load_file(TEST_CUBE);
  MB_CHK_SET_ERR(rval, "Failed to load test file");

  moab::Range all_tris;

  rval = mbi->get_entities_by_type(0, moab::MBTRI, all_tris, true);
  MB_CHK_SET_ERR(rval, "Failed to get all triangles in the model");

  MBVH BVH(mbi, all_tris);

  return 0;
  
}
