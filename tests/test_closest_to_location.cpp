

// test file locations
#include "test_files.h"

// std lib includes
#include <assert.h>
#include <ctime>
#include <iomanip>

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

#include "testutil.hpp"
#include "program_stats.hpp"

// defining 1/3 for convenience here
#define third (1.0/3.0)

/// test functions used for convenience ///

// gets all EntitySets in the MOAB instance with a Geometry Dimension Tag and a value of dim
moab::ErrorCode get_geom_sets_with_dim(moab::Interface* mbi, int dim, moab::Range& entsets);

// retrieves all volumes from the MOAB instance
moab::ErrorCode get_all_volumes(moab::Interface* mbi, moab::Range& volumes);

// retrieves all surfaces from the MOAB instance
moab::ErrorCode get_all_surfaces(moab::Interface *mbi, moab::Range& surfaces);

// retrieves all triangles from the specified volume
moab::ErrorCode get_triangles_on_volume(moab::Interface* mbi, moab::EntityHandle volume, std::vector<moab::EntityHandle>& triangles);
moab::ErrorCode get_triangles_on_volume(moab::Interface* mbi, moab::EntityHandle volume, moab::Range& triangles);

// retreieves all triangles from the specified surface
moab::ErrorCode get_triangles_on_surface(moab::Interface* mbi, moab::EntityHandle surface, std::vector<moab::EntityHandle> &triangles);

// creates TriangleRefs from MOAB triangles
std::vector<TriangleRef> create_build_triangles(moab::Interface* mbi, std::vector<moab::EntityHandle> triangles);

// returns the global id of an EntitySet
int global_id(moab::Interface* mbi, moab::EntityHandle entset);


double nrand() { return fabs(rand()/(double)RAND_MAX); }

moab::CartVect src_sample(moab::GeomQueryTool *gqt, moab::EntityHandle vol) {
  moab::CartVect min, max;
  moab::ErrorCode rval = gqt->gttool()->get_bounding_coords(vol, min.array(), max.array());
  MB_CHK_SET_ERR_CONT(rval, "Failed to get volume bounding values");

  int inside = 0;
  moab::CartVect position;  
  while(1 != inside) {
  //sample a point
  position[0] = min[0] + (max[0] - min[0])*nrand();
  position[1] = min[1] + (max[1] - min[1])*nrand();
  position[2] = min[2] + (max[2] - min[2])*nrand();

  //make sure this is inside the volume
  rval = gqt->point_in_volume(vol,position.array(), inside);
  MB_CHK_SET_ERR_CONT(rval, "Failed to determine if source location is inside the volume");
    
  }
  
  return position;
}


/// MAIN ///
int main(int argc, char** argv) {

  ProgOptions po("MOAB Robustness Tests: a program for testing robustness of the MOAB SIMD BVH against MOAB's native ray fire engine.");

  bool gqt_fire = false;
  bool eh_check = false;
  bool mem_report = false;
  bool stat_report = false;
  bool build_only = false;
  double EPS = 0.0;
  std::string filename;
  
  po.addOpt<std::string>("moab_mesh_file,f", "typically a .h5m file, this file should contain a DAGMC surface mesh", &filename);
  
  po.addOpt<void>("gqt_fire,-g", "Fire using MOAB's GQT (slower than using the OBBTree directly)", &gqt_fire);

  po.addOpt<void>("eh_check,e", "Check that EntityHandles of facets returned from the SIMD BVH match those returned from MOAB when firing at the center of triangles (only works if gqt_fire)", &eh_check);

  po.addOpt<void>("mem_rep,m", "Report memory at critical points throughout the test", &mem_report);

  po.addOpt<void>("stats,s", "Report tree structure statistics for both BVH implementations", &stat_report);

  po.addOpt<void>("build_only,b", "Only Build the BVH's and exit", &build_only);

  po.addOpt<double>("tolerance,t", "Tolerance for difference between MOAB and SIMD BVH hit distances (default is 0.0)", &EPS); 
		 
  po.addOptionHelpHeading("Options for performing robustness test");
  po.parseCommandLine(argc, argv);
  
  moab::ErrorCode rval;

  // some time-tracking values
  std::clock_t start;
  double duration = 0.0;
  
  //create a new MOAB instance
  moab::Interface* mbi = new moab::Core();

  // check for a user-specified file
  if ( filename.empty() )filename = TEST_SMALL_SPHERE;

  //load the moab file
  std::cout << "Loading file: " << filename << std::endl;
  rval = mbi->load_file(filename.c_str());
  MB_CHK_SET_ERR(rval, "Failed to load file: " << filename);
  std::cout << "Loading complete" << std::endl;
  if ( mem_report ) report_memory_usage();
  
  // initialize the GQT/GTT interfaces and build the OBBTree
  std::cout << "Building MOAB OBB Tree" << std::endl;
  moab::GeomTopoTool* GTT = new moab::GeomTopoTool(mbi, true);
  start = std::clock();
  moab::GeomQueryTool* GQT = new moab::GeomQueryTool(GTT);
  rval = GTT->construct_obb_trees();
  duration = (std::clock() - start);
  MB_CHK_SET_ERR(rval, "Failed to construct MOAB obb tree");
  // retrieve the volumes and surfaces
  moab::Range volumes;
  rval = get_all_volumes(mbi, volumes);
  MB_CHK_SET_ERR(rval, "Failed to retrieve volume meshsets");

  // working with only one volume (for now)
  assert(volumes.size() == 1);
  
  //print stats about MOAB tree
  moab::EntityHandle root_set;
  rval = GTT->get_root(volumes[0], root_set);
  MB_CHK_SET_ERR(rval, "Failed to retrieve OBB root set for volume");

  if (stat_report) {
    rval = GTT->obb_tree()->stats(root_set, std::cout);
    MB_CHK_SET_ERR(rval, "Unable to print OBBTree stats");
  }
  
  std::cout << "MOAB OBB Tree Build Complete after " << duration / (double)CLOCKS_PER_SEC << " seconds" << std::endl;
  if ( mem_report ) report_memory_usage();


  MBVHManager* MBVHM = new MBVHManager(mbi);
  std::cout << "Building SIMD BVH..." << std::endl;
  start = std::clock();
  MBVHM->build_all();
  duration = (std::clock() - start);
  std::cout << "BVH build complete after " << duration / (double)CLOCKS_PER_SEC << " seconds" << std::endl;
  if ( mem_report ) report_memory_usage();

  NodeRef* root = MBVHM->get_root(volumes[0]);
  
  if (stat_report ) {
    //print SIMD BVH stats
    BVHStatTracker *BVHS = new BVHStatTracker();
    BVHS->gatherStats(*root);
  }

  if(build_only) return rval;

  // time summations for MOAB and BVH
  double total = 0.0, moab_total = 0.0;
  
  // some stat-keeping values
  int misses = 0;
  int incorrect_distances = 0;
  int incorrect_handles = 0;
  int incorrect_triangles = 0;
  int moab_misses = 0;
  
  double accumulated_error = 0.0;
  
  int num_pnts = 100000;

  int i = 0;

  MBRay r;

  moab::EntityHandle vol = volumes[0];
  
  while(i < num_pnts) {
    i++;
    moab::CartVect origin = src_sample(GQT, vol);
    
    // fire ray
    moab::CartVect nearest_location;
    moab::EntityHandle surf;
    moab::EntityHandle facet;
    start = std::clock();
    rval = GTT->obb_tree()->closest_to_location(origin.array(), root_set, nearest_location.array(), facet, &surf);
    duration = (std::clock() - start);
    double dist = (nearest_location - origin).length();
    MB_CHK_SET_ERR(rval, "Failed in MOAB to intersect a ray with the mesh");
    moab_total += duration;

    //    std::cout << dist << std::endl;
    
    r = MBRay(origin.array(), Vec3da(0,0,0), 0.0, inf);
    
    start = std::clock();
    MBVHM->MOABBVH->intersectClosest(*root, r);
    duration = (std::clock() - start);
    total += duration;

    //    std::cout << r.tfar << std::endl;

    //    std::cout << std::endl;
    
    if (r.primID == -1) {
      misses++;
      continue;
    }
    
    if (r.geomID != surf ) {
      incorrect_handles++;
    }

    if (r.primID != facet ) {
      incorrect_triangles++;
    }

    
    // make sure the distance to hit is the same
    if( dist != r.tfar ) {
      incorrect_distances++;
      accumulated_error += fabs(dist - r.tfar);
    }

  }

  // write out information about the test
  total /= (double)CLOCKS_PER_SEC;
  moab_total /= (double)CLOCKS_PER_SEC;
  
  std::cout << "-------------------" << std::endl;
  std::cout << "Firing from origin:" << std::endl;
  std::cout << "-------------------" << std::endl;
  std::cout << "SIMD BVH" << std::endl << "--------" << std::endl;  
  std::cout << num_pnts << " took " << total << " seconds, time per query " << total/double(num_pnts) << std::endl;
  std::cout << "MOAB" << std::endl << "----" << std::endl;
  std::cout << num_pnts << " took " << moab_total << " seconds, time per query " << moab_total/double(num_pnts) << std::endl;
  std::cout << std::endl << "Ratio of MOAB time to SIMD BVH time: " << std::setprecision(3) << moab_total/total << std::endl;

  std::cout << "Missed Pnts Total: " << misses
    	    << " (" << 100*double(misses)/double(num_pnts) << "% of total pnts queried) "
	    << std::endl;

  std::cout << "Incorrect handles found (Epsilon = " << EPS << "):" << std::endl;
  std::cout << incorrect_handles
            << " (" << 100*double(incorrect_handles)/double(num_pnts) << "% of total queries) "
            << std::endl;

  std::cout << "Incorrect triangles found (Epsilon = " << EPS << "):" << std::endl;
  std::cout << incorrect_triangles
            << " (" << 100*double(incorrect_triangles)/double(num_pnts) << "% of total queries) "
            << std::endl;
  
  std::cout << "Incorrect disatnces found (Epsilon = " << EPS << "):" << std::endl;
  std::cout << incorrect_distances
            << " (" << 100*double(incorrect_distances)/double(num_pnts) << "% of total queries) "
            << std::endl;
  std::cout << "Accumulated error due to incorrect distances: " << std::setprecision(20)
	    << accumulated_error << std::endl;



  
  return misses+incorrect_distances;
  
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

std::vector<TriangleRef> create_build_triangles(moab::Interface* mbi, std::vector<moab::EntityHandle> triangles) {
  // triangle reference container
  std::vector<TriangleRef> tris;

  // create a TriangleRef for each MOAB triangle
  for(unsigned int i = 0; i < triangles.size(); i++) {
    TriangleRef *t = new TriangleRef(triangles[i], mbi);
    tris.push_back(*t);
  }

  return tris;
}

moab::ErrorCode get_all_volumes(moab::Interface* mbi, moab::Range& volumes){
  moab::ErrorCode rval;

  rval = get_geom_sets_with_dim(mbi, 3, volumes);
  MB_CHK_SET_ERR(rval, "Failed to retrieve volume sets");
  
  return rval;
}
  

moab::ErrorCode get_all_surfaces(moab::Interface *mbi, moab::Range& surfaces){
  moab::ErrorCode rval;

  rval = get_geom_sets_with_dim(mbi, 2, surfaces);
  MB_CHK_SET_ERR(rval, "Failed to retrieve surface sets");

  return rval;
}

int global_id(moab::Interface* mbi, moab::EntityHandle entset) {

  // get the entities tagged with dimension & type
  moab::ErrorCode rval;

  // get the volume id tag
  moab::Tag id_tag;
  int id; // id number of the volume

  // get the id tag handle
  rval = mbi->tag_get_handle(GLOBAL_ID_TAG_NAME, 1, moab::MB_TYPE_INTEGER, id_tag,
			     moab::MB_TAG_SPARSE);
  MB_CHK_SET_ERR_CONT(rval, "Failed to get tag with name: " << GLOBAL_ID_TAG_NAME);
  if (rval != moab::MB_SUCCESS) return -1;
  
  rval = mbi->tag_get_data(id_tag,&(entset),1,&id);
  MB_CHK_SET_ERR_CONT(rval, "Failed to get id for entityset " << entset);
  if (rval != moab::MB_SUCCESS) return -1;
  
  return id;
}

moab::ErrorCode get_triangles_on_surface(moab::Interface* mbi, moab::EntityHandle surface, std::vector<moab::EntityHandle> &triangles)
{
  // get the entities tagged with dimension & type
  moab::ErrorCode rval;  

  rval = mbi->get_entities_by_type(surface, moab::MBTRI,triangles);
  MB_CHK_SET_ERR(rval, "Failed to get triangles for surface with global id " << global_id(mbi, surface));
  
  std::cout << "Surface with global id " << global_id(mbi, surface) << " has " << triangles.size() << " triangles"  << std::endl;

  return rval;
}

/* get the triangles for the given volume */
moab::ErrorCode get_triangles_on_volume(moab::Interface* mbi, moab::EntityHandle volume, moab::Range& triangles)
{
  // get the entities tagged with dimension & type
  moab::ErrorCode rval;  

  moab::Range child_surface_sets;
  // get the child sets are all the surfaces
  rval = mbi->get_child_meshsets(volume,child_surface_sets);
  MB_CHK_SET_ERR(rval, "Failed to retrieve child surfaces of the volume");
  
  moab::Range::iterator surf_it;
  // moab ranges are additive, so it gets appended to every time
  for ( surf_it = child_surface_sets.begin() ; surf_it != child_surface_sets.end() ; ++surf_it)
    {
      rval = mbi->get_entities_by_type(*surf_it,moab::MBTRI,triangles);
      MB_CHK_SET_ERR(rval, "Failed to get triangles for surface with global id " << global_id(mbi, *surf_it));
    }
  
  std::cout << "Volume with global id " << global_id(mbi, volume)  << " has " << triangles.size() << " triangles"  << std::endl;

  return rval;
}

/* get the triangles for the given volume */
moab::ErrorCode get_triangles_on_volume(moab::Interface* mbi, moab::EntityHandle volume, std::vector<moab::EntityHandle>& triangles)
{
  moab::Range temp_tris;
  moab::ErrorCode rval = get_triangles_on_volume(mbi, volume, temp_tris);
  MB_CHK_SET_ERR(rval, "Failed to get triangles for volume");

  for(moab::Range::iterator i = temp_tris.begin(); i != temp_tris.end(); i ++) {
    triangles.push_back(*i);
  }
  
  return rval;
}
