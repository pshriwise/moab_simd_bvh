
#include "test_files.h"

#include "assert.h"

#include "MBTagConventions.hpp"
#include "moab/Core.hpp"
#include "moab/CartVect.hpp"
#include "moab/Range.hpp"
#include "moab/OrientedBoxTreeTool.hpp"

#include "MBVH.h"

#include "testutil.hpp"

#include <ctime>

#define third (1.0/3.0)

moab::ErrorCode get_all_volumes(moab::Interface* mbi, moab::Range& volumes);
moab::ErrorCode get_all_surfaces(moab::Interface *mbi, moab::Range& surfaces);

moab::ErrorCode get_triangles_on_volume(moab::Interface* mbi, moab::EntityHandle volume, std::vector<moab::EntityHandle>& triangles);

moab::ErrorCode get_triangles_on_volume(moab::Interface* mbi, moab::EntityHandle volume, moab::Range& triangles);

moab::ErrorCode get_triangles_on_surface(moab::Interface* mbi, moab::EntityHandle surface, std::vector<moab::EntityHandle> &triangles);

std::vector<TriangleRef> create_build_triangles(moab::Interface* mbi, std::vector<moab::EntityHandle> triangles);

int main(int argc, char** argv) {
  
  moab::ErrorCode rval;

  //create a new MOAB instance
  moab::Interface* mbi = new moab::Core();

  std::string filename;
  
  // check for a user-specified file
  if (argc > 1) {
    filename = std::string(argv[1]);
  }
  else {
    filename = TEST_SPHERE;
  }
    
  std::cout << "Loading file: " << filename << std::endl;

  rval = mbi->load_file(filename.c_str());
  MB_CHK_SET_ERR(rval, "Failed to load file: " << filename);

  std::cout << "Loading complete" << std::endl;

  // retrieve the volumes and surfaces
  moab::Range volumes;
  moab::Range surfaces;
  rval = get_all_volumes(mbi, volumes);
  MB_CHK_SET_ERR(rval, "Failed to retrieve volume meshsets");

  rval = get_all_surfaces(mbi, surfaces);
  MB_CHK_SET_ERR(rval, "Failed to retrieve surface meshsets");

  // working with only one volume (for now)
  assert(volumes.size() == 1);
  
  // get all of the triangles in the volume
  std::vector<moab::EntityHandle> volume_triangles;
  rval = get_triangles_on_volume(mbi, volumes[0], volume_triangles);
  MB_CHK_SET_ERR(rval, "Failed to retrieve triangles for the volume with handle: " << volumes );

  moab::Range vol_tris;
  rval = get_triangles_on_volume(mbi, volumes[0], vol_tris);
  MB_CHK_SET_ERR(rval, "Failed to retrieve triangles for the volume with handle: " << volumes ); 

  moab::OrientedBoxTreeTool* OBBTool = new moab::OrientedBoxTreeTool(mbi);


  moab::EntityHandle obb_root;
  rval = OBBTool->build(vol_tris, obb_root);
  MB_CHK_SET_ERR(rval, "Failed to build obb tree for entities " << vol_tris);

  rval = mbi->add_entities(obb_root, volumes);
  MB_CHK_SET_ERR(rval, "Failed to add geom entity sets to obb tree root");
  
  // create build triangles
  std::vector<TriangleRef> tri_refs = create_build_triangles(mbi, volume_triangles);

  TriangleBVH* TBVH = new TriangleBVH();

  BuildSettings settings;

  BuildStateTri bs = BuildStateTri(0, tri_refs);

  std::cout << "Building BVH..." << std::endl;
  NodeRef* root = TBVH->Build(settings, bs);
  std::cout << "Build complete" << std::endl;
  
  dRay r;
  Vec3da org = Vec3da(0.0, 0.0, 0.0);
  moab::CartVect origin = moab::CartVect(0.0, 0.0, 0.0);
  double location[3];
  origin.get(location);
  
  DblTriIntersector TINT;

  double ray_len = 1e17;
  
  std::clock_t start;
  double duration;
  double total = 0.0;

  int misses = 0, rays_fired = 0;
  int center_misses = 0, edge_misses = 0, node_misses = 0;

  
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

      //now prepare the different directions for firing...
      std::vector<moab::CartVect> dirs;
      
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
	TINT.intersectRay(*root, r);
	duration = (std::clock() - start)/ (double) CLOCKS_PER_SEC;
	total += duration;

	rays_fired++;

	std::vector<double> distances;
	std::vector<moab::EntityHandle> sets;
	std::vector<moab::EntityHandle> facets;

	double direction[3];
	direction[0] = this_dir[0];
	direction[1] = this_dir[1];
	direction[2] = this_dir[2];

	rval = OBBTool->ray_intersect_sets(distances, sets, facets, obb_root, 1e-3, location, direction, &ray_len);
	MB_CHK_SET_ERR(rval, "Failed in MOAB to intersect a ray with the mesh");

	MB_CHK_SET_ERR(rval, "Failed to intersect ray with set");
	
	if (r.primID == -1) {
	  
	  misses++; 

	  if ( 0 == j)
	    center_misses++;
	  else if ( j > 0 && j < 4)
	    edge_misses++;
	  else
	    node_misses++;		
	}
	
      }
  }

  std::cout << "-------------------" << std::endl;
  std::cout << "Firing from origin:" << std::endl;
  std::cout << "-------------------" << std::endl;
    
  std::cout << rays_fired << " took " << total << " seconds, time per ray " << total/double(rays_fired) << std::endl;
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

  return misses;
}


std::vector<TriangleRef> create_build_triangles(moab::Interface* mbi, std::vector<moab::EntityHandle> triangles) {

  std::vector<TriangleRef> tris;
  for(unsigned int i = 0; i < triangles.size(); i++) {

    TriangleRef *t = new TriangleRef(triangles[i], mbi);

    tris.push_back(*t);
  }
  
  return tris;
}

moab::ErrorCode get_all_volumes(moab::Interface* mbi, moab::Range& volumes){
  // get the entities tagged with dimension & type
  moab::ErrorCode rval;
  
  int three[1] = {3};
  const void* const dim[1] = {three};
  moab::Tag geom_tag;

  // get the tag handle
  rval = mbi->tag_get_handle(GEOM_DIMENSION_TAG_NAME, 1, moab::MB_TYPE_INTEGER, geom_tag,
			     moab::MB_TAG_SPARSE|moab::MB_TAG_CREAT);
  // get the entities tagged with dimension & type 
  rval = mbi->get_entities_by_type_and_tag(0,moab::MBENTITYSET,&geom_tag,dim,1,volumes);

  if (rval != moab::MB_SUCCESS )
    {
      std::cout << "Failed to get volumes from file " << std::endl;
    }
  else
    {
      std::cout << "Found " << volumes.size() << " volumes" << std::endl;
    }

  return rval;
}


moab::ErrorCode get_all_surfaces(moab::Interface *mbi, moab::Range& surfaces){
  // get the entities tagged with dimension & type
  moab::ErrorCode rval;

  int two[1] = {2};
  const void* const dim[1] = {two};
  moab::Tag geom_tag;

  // get the tag handle
  rval = mbi->tag_get_handle(GEOM_DIMENSION_TAG_NAME, 1, moab::MB_TYPE_INTEGER, geom_tag,
			     moab::MB_TAG_SPARSE|moab::MB_TAG_CREAT);
  // get the entities tagged with dimension & type 
  rval = mbi->get_entities_by_type_and_tag(0,moab::MBENTITYSET,&geom_tag,dim,1,surfaces);

  if (rval != moab::MB_SUCCESS )
    {
      std::cout << "Failed to get surfaces from file " << std::endl;
    }
  else
    {
      std::cout << "Found " << surfaces.size() << " surfaces" << std::endl;
    }

  return rval;
}


moab::ErrorCode get_triangles_on_surface(moab::Interface* mbi, moab::EntityHandle surface, std::vector<moab::EntityHandle> &triangles)
{
  // get the entities tagged with dimension & type
  moab::ErrorCode rval;

  // get the volume id tag
  moab::Tag id_tag;
  int id; // id number of the volume

  // get the id tag handle
  rval = mbi->tag_get_handle(GLOBAL_ID_TAG_NAME, 1, moab::MB_TYPE_INTEGER, id_tag,
			     moab::MB_TAG_SPARSE|moab::MB_TAG_CREAT);
  rval = mbi->tag_get_data(id_tag,&(surface),1,&id);
  

  rval = mbi->get_entities_by_type(surface, moab::MBTRI,triangles);
  
  std::cout << "Surface " << id << " has " << triangles.size() << " triangles"  << std::endl;

  return rval;
}

/* get the triangles for the given volume */
moab::ErrorCode get_triangles_on_volume(moab::Interface* mbi, moab::EntityHandle volume, std::vector<moab::EntityHandle>& triangles)
{
  // get the entities tagged with dimension & type
  moab::ErrorCode rval;

  // get the volume id tag
  moab::Tag id_tag;
  int id; // id number of the volume

  // get the id tag handle
  rval = mbi->tag_get_handle(GLOBAL_ID_TAG_NAME, 1, moab::MB_TYPE_INTEGER, id_tag,
			     moab::MB_TAG_SPARSE|moab::MB_TAG_CREAT);
  rval = mbi->tag_get_data(id_tag,&(volume),1,&id);
  

  moab::Range child_surface_sets;
  // get the child sets are all the surfaces
  rval = mbi->get_child_meshsets(volume,child_surface_sets);

  moab::Range::iterator surf_it;
  // moab ranges are additive, so it gets appended to every time
  for ( surf_it = child_surface_sets.begin() ; surf_it != child_surface_sets.end() ; ++surf_it)
    {
      rval = mbi->get_entities_by_type(*surf_it,moab::MBTRI,triangles);
    }
  std::cout << "Volume " << id << " has " << triangles.size() << " triangles"  << std::endl;

  return rval;
}

/* get the triangles for the given volume */
moab::ErrorCode get_triangles_on_volume(moab::Interface* mbi, moab::EntityHandle volume, moab::Range& triangles)
{
  // get the entities tagged with dimension & type
  moab::ErrorCode rval;

  // get the volume id tag
  moab::Tag id_tag;
  int id; // id number of the volume

  // get the id tag handle
  rval = mbi->tag_get_handle(GLOBAL_ID_TAG_NAME, 1, moab::MB_TYPE_INTEGER, id_tag,
			     moab::MB_TAG_SPARSE|moab::MB_TAG_CREAT);
  rval = mbi->tag_get_data(id_tag,&(volume),1,&id);
  

  moab::Range child_surface_sets;
  // get the child sets are all the surfaces
  rval = mbi->get_child_meshsets(volume,child_surface_sets);

  moab::Range::iterator surf_it;
  // moab ranges are additive, so it gets appended to every time
  for ( surf_it = child_surface_sets.begin() ; surf_it != child_surface_sets.end() ; ++surf_it)
    {
      rval = mbi->get_entities_by_type(*surf_it,moab::MBTRI,triangles);
    }
  std::cout << "Volume " << id << " has " << triangles.size() << " triangles"  << std::endl;

  return rval;
}
