
#include "testutil.hpp"
#include "test_files.h"

#include "MBVHManager.h"
#include "moab/Core.hpp"


moab::ErrorCode manager_test_cube();
moab::ErrorCode manager_test_sphere();
moab::ErrorCode test_manager_for_file(std::string filename);

int main(int argc, char** argv) {

  int rval;

  rval += manager_test_cube();
  rval += manager_test_sphere();

  return rval;

}

moab::ErrorCode manager_test_cube() {

  return test_manager_for_file(TEST_CUBE);

}

moab::ErrorCode manager_test_sphere() {

  return test_manager_for_file(TEST_SMALL_SPHERE);

}



moab::ErrorCode test_manager_for_file(std::string filename) {
  
  moab::Interface* mbi = new moab::Core();

  moab::ErrorCode rval = mbi->load_file(filename.c_str());
  MB_CHK_SET_ERR(rval, "Failed to load the test file: " << TEST_CUBE);
  
  MBVHManager MBVHM(mbi);

  assert(MBVHM.MDAM);

  assert(MBVHM.BVHRoots.size() > 0);
  
  moab::Tag geom_dim_tag;
  rval = mbi ->tag_get_handle(GEOM_DIMENSION_TAG_NAME, geom_dim_tag);
  MB_CHK_SET_ERR_CONT(rval, "Failed to get the geom dim tag handle");

  moab::Range surfs, vols;
  
  int dim = 2;
  void *ptr = &dim;
  rval = mbi ->get_entities_by_type_and_tag(0, moab::MBENTITYSET, &geom_dim_tag, &ptr, 1, surfs);
  MB_CHK_SET_ERR_CONT(rval, "Failed to retrieve surface entitysets");

  dim = 3;
  rval = mbi ->get_entities_by_type_and_tag(0, moab::MBENTITYSET, &geom_dim_tag, &ptr, 1, vols);
  MB_CHK_SET_ERR_CONT(rval, "Failed to retrieve surface entitysets");

  moab::Range all_sets = unite(surfs,vols);
  
  rval = MBVHM.build(all_sets);
  MB_CHK_SET_ERR(rval, "Failed to build BVH's");

  Vec3da org(0.0, 0.0, 0.0);
  Vec3da dir(0.0, 0.0, 1.0);
  
  dRay r(org, dir, 0.0, inf);
  
  rval = MBVHM.fireRay(surfs[0], r);
  MB_CHK_SET_ERR(rval, "Failed to fire ray at surface " << surfs[0]);
  CHECK(r.tfar != (double)inf);
  CHECK(r.primID != -1);
  

  
  r = dRay(org, dir, 0.0, inf);
  
  rval = MBVHM.fireRay(vols[0], r);
  MB_CHK_SET_ERR(rval, "Failed to fire ray at surface " << surfs[0]);
  CHECK(r.tfar != (double)inf);
  CHECK(r.primID != -1);
  std::cout << r << std::endl;
  CHECK(r.geomID == 10);
  
  return rval;
}
