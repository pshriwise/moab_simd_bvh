

#include "testutil.hpp"
#include "test_files.h"

#include "MBVHManager.h"
#include "moab/Core.hpp"

void backface_cull(MBRay &ray, void*) {
  moab::CartVect tri_norm(ray.Ng[0], ray.Ng[1], ray.Ng[2]);

  moab::CartVect ray_dir(ray.dir[0], ray.dir[1], ray.dir[2]);

  if(ray_dir % tri_norm > 0.0) {
    ray.geomID = -1;
    ray.primID = -1;
  }

  return;
}


int main(int argc, char** argv) {


  moab::ErrorCode rval = moab::MB_SUCCESS;


  moab::Interface* MBI = new moab::Core();

  rval = MBI->load_file(TEST_CUBE);
  MB_CHK_SET_ERR(rval, "Failed to load the test file");

  MBVHManager* BVH = new MBVHManager(MBI);
  rval = BVH->build_all();
  MB_CHK_SET_ERR(rval, "Failed to build tree(2) for the model");

  
  return rval;

}
