

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
#include "MBVH.h"

#include "testutil.hpp"

// defining 1/3 for convenience here
#define third (1.0/3.0)

// floating point epsilon check value
#define EPS 0.0

/// test functions used for convenience ///

// gets all EntitySets in the MOAB instance with a Geometry Dimension Tag and a value of dim
moab::ErrorCode get_geom_sets_with_dim(moab::Interface* mbi, int dim, moab::Range& entsets);

// retrieves all volumes from the MOAB instance
moab::ErrorCode get_all_volumes(moab::Interface* mbi, moab::Range& volumes);

// retrieves all surfaces from the MOAB instance
moab::ErrorCode get_all_surfaces(moab::Interface *mbi, moab::Range& surfaces);

// retrieves all triangles from the specified volume
moab::ErrorCode get_triangles_on_volume(moab::Interface* mbi, moab::EntityHandle volume, std::vector<moab::EntityHandle>& triangles);

// retreieves all triangles from the specified surface
moab::ErrorCode get_triangles_on_surface(moab::Interface* mbi, moab::EntityHandle surface, std::vector<moab::EntityHandle> &triangles);

// creates TriangleRefs from MOAB triangles
std::vector<TriangleRef> create_build_triangles(moab::Interface* mbi, std::vector<moab::EntityHandle> triangles);

// returns the global id of an EntitySet
int global_id(moab::Interface* mbi, moab::EntityHandle entset);


/// MAIN ///
int main(int argc, char** argv) {

  ProgOptions po("MOAB Robustness Tests: a program for testing robustness of the MOAB SIMD BVH against MOAB's native ray fire engine.");

  bool obb_fire = false;
  bool eh_check = false;
  std::string filename;
  
  po.addOpt<std::string>("moab_mesh_file,f", "typically a .h5m file, this file should contain a DAGMC surface mesh", &filename);
  
  po.addOpt<void>("obb_fire,o", "Fire using MOAB's OBBTree Directly (typically faster than using the GeomQueryTool", &obb_fire);

  po.addOpt<void>("eh_check,e", "Check that EntityHandles of facets returned from the SIMD BVH match those returned from MOAB when firing at the center of triangles (only works with obb_fire option)", &eh_check);

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

  // initialize the GQT/GTT interfaces and build the OBBTree
  std::cout << "Building MOAB OBB Tree" << std::endl;
  moab::GeomTopoTool* GTT = new moab::GeomTopoTool(mbi, true);
  start = std::clock();
  moab::GeomQueryTool* GQT = new moab::GeomQueryTool(GTT);
  rval = GTT->construct_obb_trees();
  duration = (std::clock() - start);
  MB_CHK_SET_ERR(rval, "Failed to construct MOAB obb tree");
  std::cout << "MOAB OBB Tree Build Complete after " << duration / (double)CLOCKS_PER_SEC << " seconds" << std::endl;
  
  // retrieve the volumes and surfaces
  moab::Range volumes;
  rval = get_all_volumes(mbi, volumes);
  MB_CHK_SET_ERR(rval, "Failed to retrieve volume meshsets");

  // working with only one volume (for now)
  assert(volumes.size() == 1);
  
  // get all of the triangles in the volume
  std::vector<moab::EntityHandle> volume_triangles;
  rval = get_triangles_on_volume(mbi, volumes[0], volume_triangles);
  MB_CHK_SET_ERR(rval, "Failed to retrieve triangles for the volume with handle: " << volumes );
    
  // create build triangles
  std::vector<TriangleRef> tri_refs = create_build_triangles(mbi, volume_triangles);

  // construct the SIMD BVH
  TriangleBVH* TBVH = new TriangleBVH();
  DblTriIntersector* TINT = new DblTriIntersector();
  BuildStateTri bs = BuildStateTri(0, tri_refs);
  std::cout << "Building SIMD BVH..." << std::endl;
  start = std::clock();
  NodeRef* root = TBVH->Build(bs);
  duration = (std::clock() - start);
  std::cout << "BVH build complete after " << duration / (double)CLOCKS_PER_SEC << " seconds" << std::endl;

  // initialize some ray parameters
  double origin[3] = {0.0 , 0.0, 0.0};
  double ray_len = 1e17;
  Vec3da org = Vec3da(origin);

  // time summations for MOAB and BVH
  double total = 0.0, moab_total = 0.0;
  
  // some stat-keeping values
  int misses = 0, rays_fired = 0;
  int center_misses = 0, edge_misses = 0, node_misses = 0;
  int incorrect_distances = 0;
  double accumulated_error = 0.0;
  
  std::cout << "Firing Rays" << std::endl;

  //some return values from MOAB
  double next_surf_dist;
  moab::EntityHandle surf_hit, facet_hit;

  //now prepare the different directions for firing...
  std::vector<moab::CartVect> dirs;

  
  // create a new ray struct for firing
  dRay r;
  
  for ( int i = 0; i < volume_triangles.size(); i++ ) {

    moab::EntityHandle this_tri = volume_triangles[i];

    //get the vertices of the triangle
    moab::Range verts;
    rval = mbi->get_adjacencies(&this_tri,1,0,true,verts);
    if(rval != moab::MB_SUCCESS || verts.size() != 3)
      {
	std::cout << "Error getting triangle verts." << std::endl;
	std::cout << "Verts found " << verts.size() << std::endl;
	return moab::MB_FAILURE;
      }

          //get the coordinates of the vertices
      double xs[3], ys[3], zs[3];

      rval = mbi->get_coords(verts, xs, ys, zs);
      if(rval != moab::MB_SUCCESS)
	{
	  std::cout << "Error getting vertex coordinates." << std::endl;
	  return moab::MB_FAILURE;
	}

      moab::CartVect v0(xs[0],ys[0],zs[0]);
      moab::CartVect v1(xs[1],ys[1],zs[1]);
      moab::CartVect v2(xs[2],ys[2],zs[2]);

      // clear out directions
      dirs.clear();
      //center of triangle
      dirs.push_back(unit(third*v0+third*v2+third*v2));
      //middle of edge 0
      dirs.push_back(unit(0.5*v0+0.5*v1));
      //middle of edge 1
      dirs.push_back(unit(0.5*v1+0.5*v2));
      //middle of edge 2
      dirs.push_back(unit(0.5*v2+0.5*v0));
      //at vert 0
      dirs.push_back(unit(v0));
      //at vert 1
      dirs.push_back(unit(v1));
      //at vert 2
      dirs.push_back(unit(v2));

      for (unsigned int j = 0; j < dirs.size() ; j++) {
	moab::CartVect this_dir = dirs[j];

	Vec3da dir = Vec3da(this_dir[0], this_dir[1], this_dir[2]);
	
	r = dRay(org, dir, 0.0, inf);
       
	start = std::clock();
	TINT->intersectRay(*root, r);
	duration = (std::clock() - start);
	total += duration;

	rays_fired++;

	double direction[3];
	this_dir.get(direction);

	if (obb_fire) {
	  // some extra setup for this call
	  std::vector<double> hits;
	  std::vector<moab::EntityHandle> sets, facets;
	  moab::EntityHandle root_set;
	  rval = GTT->get_root(volumes[0], root_set);
	  MB_CHK_SET_ERR(rval, "Failed to retrieve OBB root set for volume");

	  // fire ray
	  start = std::clock();
	  rval = GTT->obb_tree()->ray_intersect_sets(hits, sets, facets, root_set, 1e-03, origin, direction);
	  duration = (std::clock() - start);
	  MB_CHK_SET_ERR(rval, "Failed in MOAB to intersect a ray with the mesh");
	  moab_total += duration;

	  // if MOAB misses, but we don't, call it a win
	  if ( !sets.size() && r.primID != -1) {
	    continue;
	  }

	  if (sets.size() == 0) continue;
	  
	  // make sure that there is a hit (RIS always returns 2 hits)
	  // CHECK( sets.size() );
	  // CHECK( facets.size() );
	  // CHECK( hits.size() );

	  std::vector<double>::iterator hit_it = std::min_element(hits.begin(), hits.end());

	  next_surf_dist = *hit_it;

	  // EntityHandle check
	  if (eh_check && 0 == j) {

	    // make sure that the entity handle found by the SIMD BVH is in the list of returned facets from RIS
	    std::vector<moab::EntityHandle>::iterator eh_it = std::find(facets.begin(), facets.end(), r.eh);
	    CHECK( eh_it != facets.end() );

	    // ensure that facet hit distance matches the minimum distance found by both
	    CHECK_REAL_EQUAL( hits[eh_it - facets.begin()], next_surf_dist, EPS);
	    CHECK_REAL_EQUAL( hits[eh_it - facets.begin()], r.tfar, EPS);
	  }
	  
	}
	else{
	  // fire ray
	  start = std::clock();
	  rval = GQT->ray_fire(volumes[0], origin, direction, surf_hit, next_surf_dist);
	  duration = (std::clock() - start);
	  MB_CHK_SET_ERR(rval, "Failed in MOAB to intersect a ray with the mesh");
	  moab_total += duration;
	}

	// make sure the distance to hit is the same
	if( next_surf_dist != r.tfar ) {
	  incorrect_distances++;
	  accumulated_error = abs(next_surf_dist - r.tfar);
	}

	// add some stats if the ray misses the mesh
	if (r.primID == -1) {
	  misses++; 

	  // mark what type miss this is
	  if ( 0 == j)
	    center_misses++;
	  else if ( j > 0 && j < 4)
	    edge_misses++;
	  else
	    node_misses++;		
	}

      } // end directions loop
      
  } // end triangle loop

  // write out information about the test
  total /= (double)CLOCKS_PER_SEC;
  moab_total /= (double)CLOCKS_PER_SEC;
  
  std::cout << "-------------------" << std::endl;
  std::cout << "Firing from origin:" << std::endl;
  std::cout << "-------------------" << std::endl;
  std::cout << "SIMD BVH" << std::endl << "--------" << std::endl;  
  std::cout << rays_fired << " took " << total << " seconds, time per ray " << total/double(rays_fired) << std::endl;
  std::cout << "MOAB" << std::endl << "----" << std::endl;
  std::cout << rays_fired << " took " << moab_total << " seconds, time per ray " << moab_total/double(rays_fired) << std::endl;

  std::cout << std::endl << "Missed rays summary: " << std::endl << "----------------" << std::endl;
  std::cout << "Triangle Center Misses: " << center_misses 
	    << " (" << 100*double(center_misses)/double(rays_fired) << "% of total rays) " 
	    << " (" << 100*double(center_misses)/double(misses) << "% of missed rays) "
	    << std::endl; 
  std::cout << "Triangle Edge Misses: " << edge_misses 
 	    << " (" << 100*double(edge_misses)/double(rays_fired) << "% of total rays) " 
	    << " (" << 100*double(edge_misses)/double(misses) << "% of missed rays) "
	    << std::endl; 
  std::cout << "Triangle Node Misses: " << node_misses 
	    << " (" << 100*double(node_misses)/double(rays_fired) << "% of total rays) " 
	    << " (" << 100*double(node_misses)/double(misses) << "% of missed rays) "
	    << std::endl; 
  std::cout << "Missed Rays Total: " << misses
    	    << " (" << 100*double(misses)/double(rays_fired) << "% of total rays fired) "
	    << std::endl; 

  std::cout << "Incorrect disatnces found (Epsilon = " << EPS << "):" << std::endl;
  std::cout << incorrect_distances
            << " (" << 100*double(incorrect_distances)/double(rays_fired) << "% of total rays) "
            << std::endl;
  std::cout << "Accumulated error due to incorrect distances: " << std::setprecision(20)
	    << accumulated_error << std::endl;
   
  // if any rays miss the mesh, then this is considered a failure
  return misses + incorrect_distances;
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
moab::ErrorCode get_triangles_on_volume(moab::Interface* mbi, moab::EntityHandle volume, std::vector<moab::EntityHandle>& triangles)
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
