

#include "testutil.hpp"
#include "test_files.h"

#include "MBVHManager.h"
#include "moab/Core.hpp"

void backface_cull(MBRay &ray, void*) {
  moab::CartVect tri_norm(ray.Ng[0], ray.Ng[1], ray.Ng[2]);

  moab::CartVect ray_dir(ray.dir[0], ray.dir[1], ray.dir[2]);

  if(ray_dir % tri_norm < 0.0) {
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

  BVH->MOABBVH->set_filter_func(backface_cull);
  
  moab::Tag geom_dim_tag;
  rval = MBI->tag_get_handle(GEOM_DIMENSION_TAG_NAME, geom_dim_tag);
  MB_CHK_SET_ERR_CONT(rval, "Failed to get the geom dim tag handle");

  moab::Range surfs, vols;
  
  int dim = 3;
  void *ptr = &dim;
  rval = MBI->get_entities_by_type_and_tag(0, moab::MBENTITYSET, &geom_dim_tag, &ptr, 1, vols);
  MB_CHK_SET_ERR_CONT(rval, "Failed to retrieve surface entitysets");

  CHECK_EQUAL(1, (int)vols.size());

  MBRay ray;
  ray.org = Vec3da(-10.0, 0.0, 0.0);
  ray.dir = Vec3da(1.0, 0.0, 0.0);
  ray.instID = vols[0];
  ray.tfar = inf;
  
  rval = BVH->fireRay(vols[0], ray);
  MB_CHK_SET_ERR(rval, "Failed to fire ray");

  CHECK_REAL_EQUAL(15.0, ray.tfar, 0.0);

  std::cout << ray << std::endl;
  
  return rval;

}
