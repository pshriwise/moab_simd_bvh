
#include "test_files.h"
#include "testutil.hpp"
#include "moab/Core.hpp"

#include "MBVHManager.h"

// tolerance equal to the faceting tolerance of the test model
#define EPS 1e-03 

// retrieves all volumes from the MOAB instance
moab::ErrorCode get_all_volumes(moab::Interface* mbi, moab::Range& volumes);

// gets all EntitySets in the MOAB instance with a Geometry Dimension Tag and a value of dim
moab::ErrorCode get_geom_sets_with_dim(moab::Interface* mbi, int dim, moab::Range& entsets);

int main(int argc, char** argv) {

  moab::Interface* mbi = new moab::Core();

  moab::ErrorCode rval;

  rval = mbi->load_file(TEST_CUBE);
  MB_CHK_SET_ERR(rval, "Failed to load the test file");

  MBVHManager MBVHM(mbi);

  MBVHM.build_all();

  moab::Range vols;

  rval = get_all_volumes(mbi, vols);
  MB_CHK_SET_ERR(rval, "Failed to retrieve volumes from MOAB instance");

  Vec3da origins[6] = { Vec3da(1000.0f, 4.0f, 4.0f),
			Vec3da(0.0f, 1000.0f, 4.0f),
			Vec3da(0.0f, 4.0f, 1000.0f),
			Vec3da(-1000.0f, 4.0f, 4.0f),
			Vec3da(0.0f, -1000.0f, 4.0f),
			Vec3da(0.0f, 4.0f, -1000.0f) };
  
  Vec3da directions[6] = { Vec3da(-1.0f, 0.0f, 0.0f),
			   Vec3da(0.0f, -1.0f, 0.0f),
			   Vec3da(0.0f, 0.0f, -1.0f),
			   Vec3da(1.0f, 0.0f, 0.0f),
			   Vec3da(0.0f, 1.0f, 0.0f),
			   Vec3da(0.0f, 0.0f, 1.0f) };

  double expected_distance = 995.0f;
  
  for(size_t i = 0; i < 6; i++) {
    MBRay r(origins[i], directions[i]);
    r.instID = vols[0];

    rval = MBVHM.fireRay(vols[0], r);
    MB_CHK_SET_ERR(rval, "Failed to fire ray");

    CHECK_REAL_EQUAL(expected_distance, r.tfar, EPS);
    
    CHECK(r.tfar != (double)inf);
  }

  // cleanup
  delete mbi;
  
  return rval;
}

moab::ErrorCode get_all_volumes(moab::Interface* mbi, moab::Range& volumes){
  moab::ErrorCode rval;

  rval = get_geom_sets_with_dim(mbi, 3, volumes);
  MB_CHK_SET_ERR(rval, "Failed to retrieve volume sets");
  
  return rval;
}

moab::ErrorCode get_geom_sets_with_dim(moab::Interface* mbi, int dim, moab::Range& entsets){
  moab::ErrorCode rval;
  
  int dimension_arr[1] = {dim};
  const void* const dum[1] = {dimension_arr};
  moab::Tag geom_tag;

  // get the tag handle
  rval = mbi->tag_get_handle(GEOM_DIMENSION_TAG_NAME, 1, moab::MB_TYPE_INTEGER, geom_tag,
			     moab::MB_TAG_SPARSE);
  MB_CHK_SET_ERR(rval, "Failed to find tag with name: " << GEOM_DIMENSION_TAG_NAME);
  
  // get the entities tagged with dimension 3 & type EntitySet
  rval = mbi->get_entities_by_type_and_tag(0,moab::MBENTITYSET,&geom_tag,dum,1,entsets);
  MB_CHK_SET_ERR(rval, "Failed to retrieve geom entity sets with dimension" << dim);

  return rval;
}
