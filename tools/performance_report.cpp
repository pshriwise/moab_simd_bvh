
// test file locations
#include "test_files.h"

// std lib includes
#include <assert.h>
#include <ctime>

// MOAB includes
#include "MBTagConventions.hpp"
#include "moab/Core.hpp"
#include "moab/CartVect.hpp"
#include "moab/Range.hpp"
#include "moab/GeomTopoTool.hpp"
#include "moab/GeomQueryTool.hpp"
#include "moab/ProgOptions.hpp"

// SIMD BVH include
#include "MBVHManager.h"

#include "program_stats.hpp"
#include "rayutil.hpp"

moab::ErrorCode test_model(std::string filename);

int main(int argc, char** argv) {

  moab::ErrorCode rval;

  rval = test_model(TEST_CUBE);
  MB_CHK_SET_ERR(rval, "Model test failed");
  rval = test_model(TEST_SMALL_SPHERE);
  MB_CHK_SET_ERR(rval, "Model test failed");
  rval = test_model(TEST_SPHERE);
  MB_CHK_SET_ERR(rval, "Model test failed");

  return rval;
}

moab::ErrorCode test_model(std::string filename) {

  std::cout << std::endl;
  std::cout <<  "RUNNING TEST ON: " << filename << std::endl;
  moab::ErrorCode rval;

  // some time-tracking values
  std::clock_t start;
  double bvh_build_duration = 0.0, obb_build_duration = 0.0;
  double bvh_ray_fire_duration = 0.0, obb_ray_fire_duration = 0.0;

  //create a new MOAB instance
  moab::Interface* mbi = new moab::Core();

    //load the moab file
  rval = mbi->load_file(filename.c_str());
  MB_CHK_SET_ERR(rval, "Failed to load file: " << filename);

  double mem_after_load = report_memory_usage();
  if (mem_after_load < 0.0 ) return moab::MB_FAILURE;
  
    // initialize the GQT/GTT interfaces and build the OBBTree
  std::cout << "Building MOAB OBB Tree" << std::endl;
  moab::GeomTopoTool* GTT = new moab::GeomTopoTool(mbi, true);
  start = std::clock();
  moab::GeomQueryTool* GQT = new moab::GeomQueryTool(GTT);
  start = std::clock();
  rval = GTT->construct_obb_trees();
  obb_build_duration = (std::clock() - start);

  double mem_after_obb = report_memory_usage();
  if (mem_after_obb < 0.0 ) return moab::MB_FAILURE;
  
  std::cout << "Estimated size of the OBBTree: " << mem_after_obb - mem_after_load << "MB" << std::endl;
  
  MB_CHK_SET_ERR(rval, "Failed to construct MOAB obb tree");
  // retrieve the volumes and surfaces
  int dimension_arr[1] = {3};
  const void* const dum[1] = {dimension_arr};
  moab::Tag geom_tag;

  // get the tag handle
  rval = mbi->tag_get_handle(GEOM_DIMENSION_TAG_NAME, 1, moab::MB_TYPE_INTEGER, geom_tag,
			     moab::MB_TAG_SPARSE);
  MB_CHK_SET_ERR(rval, "Failed to find tag with name: " << GEOM_DIMENSION_TAG_NAME);
  moab::Range volumes;  
  // get the entities tagged with dimension 3 & type EntitySet
  rval = mbi->get_entities_by_type_and_tag(0,moab::MBENTITYSET,&geom_tag,dum,1,volumes);
  MB_CHK_SET_ERR(rval, "Failed to retrieve geom entity sets with dimension" << dimension_arr);

  // working with only one volume (for now)
  assert(volumes.size() == 1);

  //print stats about MOAB tree
  moab::EntityHandle root_set;
  rval = GTT->get_root(volumes[0], root_set);
  MB_CHK_SET_ERR(rval, "Failed to retrieve OBB root set for volume");
  
  MBVHManager* MBVH = new MBVHManager(mbi);

  start = std::clock();
  rval = MBVH->build(volumes);
  bvh_build_duration = (std::clock() - start);
  MB_CHK_SET_ERR(rval, "Failed to build the BVH for volume: " << volumes[0]);
  
  double mem_after_bvh = report_memory_usage();
  std::cout << "Estimated size of the SIMD BVH: " << mem_after_bvh - mem_after_obb << " MB" << std::endl;
  
  MBRay ray;
  moab::CartVect org, dir;
  org = moab::CartVect(0.0, 0.0, 0.0);

  double origin[3] = {0.0, 0.0, 0.0};
  
  size_t num_rays = 1E6;
  std::vector<double> hits;
  std::vector<moab::EntityHandle> sets, facets;

  for(size_t i = 0; i < num_rays; i++){
    RNDVEC(dir);

    ray = MBRay(org.array(), dir.array());
    ray.instID = volumes[0];

    start = std::clock();
    rval = MBVH->fireRay(ray);
    bvh_ray_fire_duration += (std::clock() - start);
    MB_CHK_SET_ERR(rval, "Failed to fire ray " << ray << " on BVH");

    double direction[3];
    direction[0] = dir[0]; direction[1] = dir[1]; direction[2] = dir[2];
    start = std::clock();
    rval = GTT->obb_tree()->ray_intersect_sets(hits, sets, facets, root_set, 1e-03, origin, direction);
    obb_ray_fire_duration += (std::clock() - start);
    MB_CHK_SET_ERR(rval, "Failed to fire ray " << org << dir << " on OBB tree");
  }

  std::cout << "Build time ratio: " << obb_build_duration/bvh_build_duration << std::endl;
  std::cout << "Ray fire ratio: " << obb_ray_fire_duration/bvh_ray_fire_duration << std::endl;

  delete mbi;
  delete MBVH;

  std::cout << std::endl;
  
  return rval;
}
